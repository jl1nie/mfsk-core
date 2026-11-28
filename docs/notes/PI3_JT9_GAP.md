# Pi 3B (P2): how close to `jt9` the cells flow gets, and what stops it (host study, 2026-11-28)

**No Pi 3 was measured.** Every figure is from this x86 host (the cells flow, `fixed-point`, 8 kHz; the faithful `Decoder<Ft8>`, f32). Pi 3B time is an extrapolation (`PROFILES.md` section 8: 1.0-1.5 ns a host instruction); the host's wall times below are for ratios only. Tools: `scripts/pi3-sweep.sh` / `.py`, `mfsk-core/examples/pi3_sweep.rs` (`probe`, `probe2`, `probe3`, `probe4`). Corpus: 22 real recordings (jt9's 402) and 38 generated busy slots (their truth, 936): 60 slots, 1 338 references.

## 1. Recall against cost (60 slots; host ms a slot, one thread)

| configuration | decodes of the 1 338 | 22 recordings (of jt9's 402) | extras | ms a slot |
|---|---|---|---|---|
| cells, 1 pass, OSD (30-300 candidates) | 74.3 % | 71.1 % | 38 | 38-40 |
| cells, SIC 2 passes, OSD | 88.2 % | 84.3 % | 73 | 73 |
| cells, SIC 3 passes, OSD | 88.9 % | 84.8 % | 75 | 75 |
| **cells, SIC 3-4 passes, OSD, 60-300 candidates** | **89.8 %** | **85.1 %** | 79-80 | 79-80 |
| faithful Normal / Deep, AP off | 93.0 % / 97.1 % | 90.8 % / 95.3 % | 86 / 115 | 199 / 358 |
| faithful Deep, AP full (the ceiling) | 97.8 % | 97.8 % | 132 | 392 |
| cells SIC 3 + OSD, then faithful Deep (union) | 99.2 % | 98.8 % | 154 | 475 |

* **CORRECTED (section 13): the slot is a constraint for the faithful pipeline on a Pi 3B.** Faithful Deep is 0.4 s a slot (worst 0.5 s) on this host, but this host runs a host instruction in about 0.04 ns (13 050 M instructions in 0.5 s), a Pi 3B in 1.0-1.5 ns (`PROFILES.md` section 8): **25-40 times slower, Deep 10-15 s a slot on one core, Normal 5-8 s**, not "a few seconds". **Threads do not help** as `PROFILES.md` assumed (60-80 % efficiency over 4 cores): 4 threads gave 1.07x (faithful) and 1.0x (cells) here.
* **The cells flow plateaus at 89.8 %**: 60, 100 or 300 candidates and 3 or 4 passes give the same 1 201 decodes.

## 2. Where the cells flow's misses are (60 of jt9's 402 on the 22 recordings)

| hypothesis | measurement | result |
|---|---|---|
| the search misses them | a candidate within 4 Hz / 0.17 s of jt9's place in the 300-candidate list | **no**: 50 of 60 have one |
| the place is wrong | decode at jt9's exact place and 8 neighbours (+-3.125 Hz, +-0.08 s), OSD | **no**: 0 of 60 |
| the cells (8 kHz, half-dB, rows of 80 ms) lose symbol information | hard-decision tone errors of 79, 12 kHz DFT at the exact place against the cells | **no**: 43.0 against 42.7 for the misses (found ones: 17.8 against 14.8) |
| other signals interfere | subtract every other reference signal (jt9's messages, frequency and DT refined), decode again | **barely**: 3 of 60; errors 43.0 -> 40.8; on `jt9`'s own residual audio 35.7 |
| the SIC's subtraction is shallower (phase error) | each decoded signal's own tone energy left, cells against the faithful `subtract_tones_lpf_refine_dt`, 274 signals | **no**: -13.9 / -16.7 / -19.7 / -21.3 / -24.9 dB against -14.1 / -16.0 / -20.8 / -22.9 / -26.7 (SNR bands); `ft8dec_sic.rs` `depth_against_the_faithful_subtraction` guards it |
| the decoder is broken | AWGN and CCIR fading sweeps (`ft8_sweep`, 20 slots a SNR), cells + OSD against faithful | **no**: 50 % threshold about -20.5 dB against Normal's -20.5 and Deep's -21.5: **within 1 dB** (faithful Deep's coherent `llrb`..`llre` variants are worth about 1 dB) |

**What `jt9` does with them** (the trace build, `ft8_trace.dat`; `jt9 -8 -d 3`): of the 59 misses it has a decode line for, **81 % were decoded by the OSD** (`ntype` 2), against 22 % of the ones the cells flow finds; the codeword differs from the hard decisions in 20.1 bits on average (11.5 for the found ones). By LLR variant: `llra` (nsym 1, power only: the cells flow has it) 27, `llrb` 9, `llrc` 8, `llrd` 7, `llre` 4, AP 4. AP is not the cause (124 of the faithful pipeline's 128 extra decodes are found with AP off).

## 3. What is not explained

`jt9` decodes these at about 11 % bit errors, where a hard decision on the symbols at its printed place gives about 45-54 % symbol errors, **in the 12 kHz audio as well as in the cells**. Whatever `jt9` receives is far better than any measurement made here, and 27 of the 59 are decoded from `llra` + OSD, which needs no phase. What differs is not found: the next experiment is to dump `jt9`'s `llrz` (`ft8b.f90` before `decode174_91`) for these signals and compare it bit for bit with the cells flow's LLR at the same place; whether it is the fine synchronisation (0.1 Hz, 5 ms against the cells' 3.125 Hz / 80 ms grid with interpolation), the 12 kHz baseband, or a defect in the cells flow's LLR for weak signals is not known. **Phase (`im: 0`) cannot be the whole story** (27 of 59 need none) and could be part of it (17 decoded by `llrb`/`llrc`).

## 4. What a real Pi 3 must measure first
The ns a host instruction on an A53 (and the NEON FFT), whether rayon helps at all (the host says it does not), the working set against the 512 KB L2.

## 5. What the LLR dump of `jt9` showed (2026-11-28, later the same day)

The trace build of `jt9` (`target/upstream/wsjtx-v3.2.0-rc1-trace`, a local copy, not committed) was given one more dump at each successful decode: `llra`..`llre`, `llrz`, `s8` (the 8 x 79 tone magnitudes) and a checksum of `dd0`. The cells flow's LLR at the same place (`fill` of the grid place, `compute_llr_fast_metric`, the 3 x 3 places round it; the patch of that measurement is `llr_against_jt9.patch`, kept out of the tree: it cannot fail) against `jt9`'s `llra`, bit errors of the signs against the transmitted codeword (of 174):

| | n | `jt9` `llra` | cells at `jt9`'s place | cells, best of 3 x 3 | correlation |
|---|---|---|---|---|---|
| found by the cells flow | 326 | 15.8 | 18.1 | 17.1 | 0.81 |
| **missed** | 57 | **30.3** | **56.6** | 53.5 | 0.44 |

* For the ones the flow finds the two LLR agree (0.81). For the misses `jt9`'s LLR is far cleaner than the cells' (many at 7-14 bit errors where the cells have 35-70); a search of +-8 rows and +-6 bins finds no offset that fixes it (best error 50.8 on average, 20 of 57 under 40): **the place is not the cause**.
* **One case traced to the end, `CU2DX SP6DXH -19`** (`websdr_test7`, 527 Hz, decoded by `jt9` at stage 50, pass 1, BP, 8 hard errors): `jt9`'s own `s8` reads the 79 tones correctly from **symbol 14 on** (the argmax sequence equals the transmitted one from there) and has the first 14 symbols destroyed by a constant tone 7, which is `RA6FSD SP2EWQ -07` at 570.4 Hz (+43.75 Hz = its tone 7), +17 dB, decoded by `jt9` at stage 41 (and by the cells flow). The same audio read the same way here (a 12 kHz DFT, or `ft8_downsample`'s baseband reimplemented, validated on strong signals: 0-3 errors) shows that tone 7 over **many symbols of the whole message** (32 of 79 symbols wrong): **`jt9` reads the weak signal from audio from which the strong neighbour has been subtracted, this recording's neighbour is not removed in what I measure.** `jt9` decodes in up to 3 stages (nzhsym 41, 47, 50) of 3 passes, each stage re-reading the audio and subtracting what the earlier stages found; the trace's `resid_*.bin` are the audio after a pass, and the **audio at the first pass of stage 50 is not the raw audio** (the `probe4` figure for those signals, which used the raw audio, was too pessimistic).
* So the open question is narrower: after the cells SIC subtracts `RA6FSD` (found, +17 dB), why is `CU2DX` not decoded in a later pass, when `jt9` reads it cleanly from symbol 14? The oracle (every other reference signal subtracted with this crate's `refine_freq` + `subtract_tones_lpf_refine_dt`) gave 3 of 60; `jt9` subtracts with the exact (sub-sample, sub-0.1 Hz) parameters of its own fine sync and amplitude tracking. Next measurements: the cells flow's own pass-2 candidate list and LLR at `CU2DX`'s place on **its** residual cells (needs a hook in `ft8dec_sic.rs`), and `jt9`'s residual (`resid_41_*`) fed to the cells flow.

## 6. The algorithm differences: the cells flow's SIC against `jt9`'s staged decode (2026-11-28)

**Where `jt9` decodes what the cells flow misses.** The trace's stage and pass of each signal's first decode (`PASS` lines of `ft8_trace.dat`): of the 342 the cells flow finds, 78 % were decoded by `jt9` at stage 41, pass 1 (the first look), 16 % in pass 2 or 3 of their stage; of the **60 it misses, 17 % in the first look, 28 % in passes 2-3 of stage 41, 54 % at stage 50** (the last stage, whose audio has the earlier stages' decodes subtracted): **about 8 in 10 are decoded by `jt9` only after something has been subtracted**. `jt9` searches the whole band afresh in every pass, 300-400 candidates a pass, three passes in each of up to three stages (nzhsym 41, 47, 50); the cells flow (`ft8dec`'s way) searches only within +-18 cells (56 Hz) of what the last pass subtracted, at most `max_cand / 2` candidates (8 at least), hard sync at least 7.

**The experiment** (switches in `ft8dec_sic.rs`, since removed: patch `sic_experiment_and_after_subtraction.patch` in the session scratchpad): later passes over the **whole band** (A), **no cap** on the candidates of a later pass (B), both, and with the hard-sync minimum at 0, at 3 and 4 passes, `cells:sicNosd:c100`, 60 slots:

| | hits of the 1 338 | 22 recordings of jt9's 402 | extras |
|---|---|---|---|
| shipped (3 passes / 4 passes) | 1 201 / 1 201 (89.8 %) | 342 / 342 | 79 / 80 |
| A whole band | 1 201 / 1 202 | 342 / 343 | 83 / 84 |
| B no cap | 1 201 / 1 201 | 342 / 342 | 79 / 80 |
| A + B (+ hard sync 0) | 1 201 / 1 202 | 342 / 343 | 83 / 84 |

**The restriction of the later passes is not what holds the plateau**: the whole-band search and the uncapped passes find nothing more (+0 to +1) and cost +4 extras and 40-60 % more time.

**What the subtraction does to a weak signal** (an `#[ignore]`d measurement in `ft8dec_sic.rs`: `llra` bit errors of 174 at `jt9`'s place, on the original cells and on the residual cells after pass 0 has been subtracted): `CU2DX` 45 -> 36 (`jt9`: 8), `SB7W` 69 -> 38, and **no change at all** for `SV2FPI` (64; `jt9` 7), `DL2HRE` (37; 17), `EA8PP` (59; 9), `UB3AQS` (69; 12): nothing near them was subtracted, so **interference is not what stops these**. The cells' hard decision per symbol at the grid place (`.` right, `x` wrong), e.g. `SV2FPI`: `xxxxxxx..xxxxxx.xxxxxxx..x....xx.xxxxxx...xxxxxxxx.......x.xxxxx.xxx.....xxxxxx`: **the errors come in runs**, and in five of the six the **first 7-14 symbols are wrong and the last 12-15 are right** (`CU2DX`: `x..xxxxxxx.xxx.xx......xx.x....xx..xxxxxx...xxxx...xx.xxxxxx.x.x...............`). `jt9`'s own `s8` for `CU2DX` and `SV2FPI` shows the same shape (the first Costas array and the first data symbols wrong) with the **data symbols mostly right** (6 of 58 and 10 of 58 wrong). A signal that is strong in its second half and wrecked in its first: fading, drift, or an interferer that is there at the start. Not yet separated; the next measurement is the tone power of the right tone against the neighbours, symbol by symbol, in the 12 kHz audio and in the cells, for these six.

## 7. How the misses are made: the time course of their tones (2026-11-28)

The subtraction's depth by symbol position (`depth_against_the_faithful_subtraction`, strong signals, median dB left): symbols 0-6 -20.0 dB (faithful -20.2), 7-13 -24.7 (-28.1), 14-35 -23.4 (-27.6), 36-71 -23.0 (-27.2), 72-78 -22.6 (-24.0): **nothing special at the start**; the cells flow is 3-4 dB shallower in the middle (not the +-1.8 dB of the all-symbol mean).

How many of the 60 misses have a strong neighbour (jt9's SNR, within +-50 Hz and +10 dB above): **19 (32 %)**, against 31 of the 342 found (9 %): interference is a real factor and not the main one (the other 41 have none).

**The tones' time course** (`pi3_sweep probe7`: in the 12 kHz audio, at the best of +-0.5 Hz x +-0.03 s, the hard-decision errors and the right tone's power over the others in six runs of symbols), 58 of the misses against 325 found:

| class | missed | found |
|---|---|---|
| clean in the raw audio (< 25 % errors) | **8 (14 %)** | 232 (71 %) |
| early bad, late better | 12 (21 %) | 23 (7 %) |
| late bad, early better | 8 (14 %) | 28 (9 %) |
| bursty / mid-message | 21 (36 %) | 31 (10 %) |
| moderate, flat (25-40 %) | 5 (9 %) | 6 (2 %) |
| uniformly bad (>= 40 %) | 4 (7 %) | 5 (2 %) |

Mean error rate of the misses is 0.47-0.54 in every run (the found: 0.15-0.24), the right tone's power 4-6 dB over the others (the found: 11-12 dB): **the misses are weak, 7 dB weaker than the found ones, and uneven in time** (the thresholds are crude: the runs are 13 symbols). **CORRECTED in section 8: the "8 clean in the raw audio" group and the "cells-specific loss" read from it were a selection effect (the winner's curse of a best-of-35 search), not a finding.** The rest is signals `jt9` reads with an LLR 5-10 times cleaner than the cells' at the same place, which is **not explained**.

## 8. Correction: the "clean in the raw audio" group was a selection effect (2026-11-28)

Section 7's 12 kHz errors are the **minimum over 35 places** (+-0.5 Hz x +-0.03 s); a count that is the minimum of 35 noisy counts is low by chance, and a group chosen by it ("12 kHz best < 25") is the winners of that chance. The cells' errors at that place were not chosen. Compared fairly (the same 35-place search on the 8 kHz audio, a one-symbol rectangular window; `pi3_sweep probe8`), hard-decision tone errors of 79:

| | 12 kHz best of 35 | 8 kHz rect best of 35 | 8 kHz rect at the 12 kHz place | the cells' design in `f64` (Hann over two symbols), grid | its best of 3 x 3 |
|---|---|---|---|---|---|
| found (325) | 14.3 | **14.0** | 15.9 | 15.9 | 14.7 |
| missed, all (58) | 37.6 | 41.4 | 44.8 | 44.9 | 42.4 |
| missed, "clean" by the 12 kHz search (17) | 17.5 | 28.7 | 32.0 | 32.6 | 30.9 |

* **The 8 kHz audio, the Hann window over two symbols and the 80 ms / 3.125 Hz grid lose nothing for the signals the flow finds** (14.3 / 14.0 / 14.7), and about 4 errors of 79 on the misses (37.6 against 41.4; it is noisy: the standard error is about 2). The **fixed-point transform is not a cause**: the cells' design in `f64` has the same errors as the shipped cells (32.6 against 32.7 for the 17, 15.9 against 15.8 for the found). **Interpolating at the true fractional place does not help** (32.5 against 32.7 for the 17). **Clipping** cannot be it (the `websdr` slots peak at 8-14 k, the `191111` ones at 22-32 k, and both are among the misses), nor the transform's **gain** (shift 0 in every slot). A gap by frequency shows no aliasing pattern (the 12 kHz -> 8 kHz conversion folds only 4-6 kHz into 2-4 kHz).
* So **there is no separate class of 8 signals the cells flow reads worse than the raw audio**: the group of section 7 is the winners of a search. What is left is one population, **weak and uneven signals that `jt9` decodes and that no hard-decision measure here reads** (12 kHz best-of-35: 37.6 of 79 wrong), with an LLR (`jt9` 30.3 bit errors of 174 against the cells' 56.6) that is far cleaner than the same audio gives in any form tried. For `CU2DX` `jt9`'s own `s8` has 6 of 58 data symbols wrong where the best 12 kHz reading has about 16: not explained by selection (`jt9` chooses its place by the Costas symbols).
* **What is left to try** (not yet done): feed the cells flow's decoder `jt9`'s own `llra` (the dump) to see whether the BP / OSD of this crate decodes what `jt9` decodes from it (separates the decoder from the symbol energies); and a coherent look at the signals at the best place, with phase.

## 9. Correction: `jt9` prints DT to 0.1 s; the oracle was void; with the precise places the subtraction is the lever (2026-11-28)

**A measurement flaw.** `truth.tsv` and `tests/fixtures/ft8_lib_jt9_beta6.tsv` carry the DT `jt9` prints, to **0.1 s** (and the frequency to 1 Hz): the true DT is off by up to 50 ms (the trace's precise values differ from the printed ones by 0.024 s on average, 0.050 s at most, 0.24 Hz): a third of a symbol. A subtraction reference built at that place removes nothing (a strong signal left at 0.0 dB; 31 signals subtracted, energy left -0.2 dB). **The oracle of section 2 (every other reference signal subtracted, "3 of 60 decode") was void** for this reason, and the places of `probe`, `probe3`, `probe5`-`probe8` were the printed ones (the best-of-35 searches of 0.03 s only partly make up for it). What used the trace's precise `f1` / `xdt` (the LLR comparison of section 5, the `jt9`-LLR-into-our-decoder measurement) stands. `ft8_trace.dat`'s `DEC` lines are the precise places (`truth2.tsv` in the scratchpad was built from them: 401 of 402).

**Redone with the precise places:**

| | result |
|---|---|
| subtraction of the other 31 signals of `websdr_test7` (strongest first, `refine_freq` + `subtract_tones_lpf_refine_dt`) | each strong signal's own tone energy left -22.7 .. -29.5 dB, the slot's energy **-9.8 dB** (`jt9`'s own `dd0` at the decode of `CU2DX`: **-12.2 dB**) |
| the cells flow (1 pass, no OSD) on that residual | decodes `CU2DX SP6DXH -19`, which it misses on the raw audio |
| the cells flow (1 pass, no OSD) on **`jt9`'s own `dd0`** at that decode (stage 50, pass 1; the dump of the trace build) | decodes it too; this crate's reading of `dd0` (`ft8_downsample`'s baseband, 32-point DFT) gives 12-13 wrong of 79, `jt9`'s own `s8` 12 |
| **all 60 misses**, every other reference signal subtracted (precise places), cells flow with the OSD | **24 of 60 (40 %)** decode (the whole flow and decoding at the true place agree): jt9 SNR -30..-18 dB 3 of 10, -18..-12 dB 6 of 18, -12..-6 dB 7 of 13, -6..0 dB 6 of 11, 0+ dB 2 of 8 |

**So the subtraction is a lever after all**: what `jt9` reads is the audio after the earlier stages' decodes have been removed (the `dd0` of the first pass of stage 50 is 12 dB under the recording: 99 % of the energy). The cells flow decodes the weak signals from that audio. What is not known is how much of the 40 % the cells flow's **own** SIC can reach: its subtraction removes only the signals **it decoded** (21 of the 31 in `websdr_test7`) and each of those needs the ones before it (a chain), where the oracle removes all; its depth per signal is about 3-4 dB shallower than the faithful one (section 7). Next measurements: (a) the oracle restricted to the signals the cells flow itself decodes in pass 0 (the set), subtracted with `jt9`'s precise parameters (the quality of the subtraction not at issue): separates "which signals" from "how well"; (b) the same with the cells flow's own estimated places. The earlier sections' conclusions that depend on the void oracle ("interference is not the cause") are withdrawn; the section-2 table's "other signals interfere: barely" line is wrong.

## 10. Which signals to subtract, and how well (2026-11-28)

The 60 signals of `jt9`'s the cells flow misses (one pass, OSD, 100 candidates on the slot, then one more plain pass on the residual; `jt9`'s precise places from the trace; `pi3_sweep probe11`, and `cells_subtraction_at_jt9s_places` in `ft8dec_sic.rs`, kept as a patch in the session scratchpad: it cannot fail), how many are decoded on the residual:

| what is subtracted, with what, at which place | decoded of 60 |
|---|---|
| every other reference signal, the faithful subtractor (`refine_freq` + `subtract_tones_lpf_refine_dt`), `jt9`'s precise places (the oracle) | **24** |
| only what the cells flow decoded in pass 0, the faithful subtractor, `jt9`'s precise places | **16** |
| only what the cells flow decoded in pass 0, the faithful subtractor, **the places the cells flow estimated** | 1 |
| only what the cells flow decoded in pass 0, **the cells flow's own subtractor** (`ResidualSource`), `jt9`'s precise places | **2** |
| the same, at the places the cells flow estimated (its own SIC, pass 2) | 0 |

* **The set** (subtracting the undecoded signals too) is worth 8 of the 24. **The subtractor and its places** are worth the rest: 16 with the faithful subtractor at precise places against 1 at the cells flow's estimates and 2 with the cells flow's own subtractor at precise places. **The cells flow's subtraction is where a third of the misses are** (16 of 60: the faithful subtractor, the decoded set, precise places).
* The per-signal depth of section 6 / `depth_against_the_faithful_subtraction` (the signal's own tone energy left: -23 dB against -27 dB) does **not** predict this: what matters for the decode of a weak neighbour is not the energy left at the signal's own tones. Candidates, not measured: the faithful subtractor refines the frequency (0.1 Hz) and the DT on the audio itself before subtracting; the cells flow's `ResidualSource` takes its places from the cells (a parabola on 3.125 Hz / 80 ms) and corrects them while streaming; the faithful subtractor works on 12 kHz audio with the GFSK shape, the cells flow's on 8 kHz; the residual's spectral spread (what a weak signal 10-50 Hz away sees) is not what the own-tone energy measures. **Next measurements:** the residual's energy in the band 10-60 Hz round each subtracted signal (excluding the signal's own tones) for both subtractors on the same signals; and the cells flow's subtractor with a frequency refinement of the faithful kind (0.1 Hz, on the 8 kHz audio) in front of it.

The same measurement with a fourth row (`probe11`, (d)): the cells flow's pass-0 decodes at the places it estimated, **refined on the 12 kHz audio before the faithful subtractor** (for each DT of +-60 ms in 5 ms steps the frequency by `refine_freq` at 0.1 Hz, the pair with the largest tone correlation kept):

| what is subtracted, at which place | decoded of 60 |
|---|---|
| the cells flow's pass-0 decodes, **estimated places** | 1 |
| the same, **refined on the 12 kHz audio first** | **14** |
| the same, `jt9`'s precise places | 16 |
| every other reference signal, `jt9`'s precise places (the oracle) | 24 |

**The cells flow's estimate of a decoded signal's place (about 0.4 Hz and 28 ms) is too coarse to subtract it; refined on the 12 kHz audio it is almost as good as `jt9`'s (14 against 16).** 14 of the 60 misses (23 %) come back from one refinement of the places of what the flow already decodes. The 10 more of the oracle are the undecoded signals. Next: a cells SIC that holds the 12 kHz audio, refines and subtracts there and makes the cells again, measured on the 60 slots (not an estimate from the misses).

## 11. The causes, taken apart on `jt9`'s own audio (2026-11-28)

`jt9` was run with a dump of its audio (`dd0`) at the decode of each of the signals the cells flow misses (an environment variable of the trace build picks the message; 59 of 60 dumped). Everything below is on **that audio**, so the subtraction history is the same for `jt9` and for us:

| from `jt9`'s audio at the decode, of the 59 misses | decoded |
|---|---|
| the cells flow (1 pass, no OSD / OSD, 3 passes) | 8 / **14** |
| the faithful `deep:off` | **43** |
| the cells flow, **decoding at `jt9`'s precise place** (OSD) / any of the 9 places | 16 / 21: the search is not what fails |
| hard-decision tone errors of 79 at that place: `jt9`'s own `s8` / a 12 kHz rectangular DFT / the cells | 25.9 / 35.2 / 35.7 (medians 25 / 28 / 30): for 47 of 57 the three agree; for **10 `jt9`'s `s8` is 20 or more errors cleaner** than the DFT at the same place |
| `llra` bit errors of 174: `jt9`'s / **this crate's LLR function applied to `jt9`'s `s8`** | 30.3 / 30.5 (and 22 of 57 decoded by this crate's BP + OSD from either): **the LLR formation is the same** |

**What this crate's BP + OSD decodes from `jt9`'s own LLR variants** (53 of the misses; `llra`, `llrb`, `llrc`, `llrd`, `llre`): 22 / 27 / 27 / 26 / 31 alone; the union of **`a` + `d` (what magnitudes alone can make) 30, + `b` + `c` (coherent over two and three symbols, which need phase) 42, + `e` 44 (83 %)**. For the ones the cells flow finds the union of `a` + `d` is 292 of 319 and of all five 296: the coherent variants matter only for the weak ones.

**So, for the misses** (53, on `jt9`'s audio): 44 can be decoded from `jt9`'s LLR; 30 from magnitude-only variants of `jt9`'s symbol energies (what a cells flow without phase could reach at best); the cells flow decodes 14.
* **A. No phase** (`im: 0`): the coherent variants are worth about **14** (30 -> 44). The comment of `fill_mask` (phase worth at most 3 of 1 338, `FT8_WINDOW_AND_COARSE_DESIGN.md` section 12) does not hold for the weak signals that `jt9` decodes and the cells flow misses.
* **B. The symbol energies**: about **16** (30 -> 14), among them the 10 signals whose `s8` is far cleaner than a DFT at the same place. **Not explained**: `jt9`'s front end (`ft8_downsample`: a 192 000-point FFT of the slot, the band cut with cosine tapers, `twkfreq` and the 0.5 Hz / 5 ms fine search of `ft8b`) gives the cleaner energies; `probe6` reproduces its `s8` (12-13 wrong of 79 on `CU2DX`'s `dd0`), so the difference can be taken apart step by step.
* **C. The subtraction** (section 10): 14 of the 60 from refining the places of what is decoded.
* The rest (9): `RR73` and hashed-call messages, whose transmitted bits this study builds with `pack77` and may not equal.

## 12. Correction of section 11: the dump was taken at the wrong decode; the front end taken apart (2026-11-28)

**A flaw in the dump of section 11.** `jt9` decodes a message again in the same pass when two candidates land on one signal (10 of the 59 messages twice, one three times), and `subtractft8` removes the signal from `dd0` after the first decode; the dump (an environment variable of the trace build) was overwritten at each decode, so for those 11 it held the audio **after the signal had been subtracted** (noise at that place: 70-79 of 79 tones wrong, while `jt9`'s own `s8`, made from the FFT it cached at the start of the pass, had 22). The "10 signals whose `s8` is 20 or more errors cleaner than any reading" were this artefact. Redone with the dump at the **first** decode (59 of 60), `jt9`'s precise places:

| from `jt9`'s audio at the first decode, of the 59 misses | decoded |
|---|---|
| the cells flow, 1 pass / OSD, 3 passes | 10 / **18** |
| the cells flow decoding at `jt9`'s precise place (OSD) / any of 9 places | 20 / 27 |
| the faithful `deep:off` | **52** |

Hard-decision tone errors of 79 at `jt9`'s place (mean / median, 57 signals): `jt9`'s own `s8` **25.9 / 25**; a 12 kHz rectangular DFT 27.5 / 27 (the symbols 5 ms later: 27.4); **the audio first cut to `ft8_downsample`'s band (kept at 12 kHz) 25.8 / 25**; **the whole of `ft8_downsample` (shift, 200 Hz, 32-point DFT) 25.8 / 25**; the cells (best of 3 x 3) **29.8 / 29**. Two signals only are 10 or more errors cleaner in `jt9`'s `s8` than in the DFT.
* **The front end of `jt9` is reproduced** (`pi3_sweep probe12`: 25.8 against 25.9). Of the 3.9 errors the cells lose against it, **1.7 are the band cut** (the filter of `ft8_downsample`: out-of-band energy removed before the symbols are read) and **2.3 the cells' representation** (8 kHz, Hann over two symbols, half-dB bytes, the grid). Small in errors, large in decodes: the decode is a steep function of the error count near its threshold.

**The causes, on `jt9`'s audio, of the misses** (this crate's BP + OSD on `jt9`'s LLR variants: `a` + `d` 30 of 53, all five 44; the cells flow decodes 18 of 59 and the faithful pipeline 52 of 59): **A** the coherent variants (`b`, `c`, `e`: they need phase, which the cells do not keep): 30 -> 44, about **14**; **B** the cells' symbol energies 3.9 errors worse than `jt9`'s: 30 -> 18, about **12**; **C** the subtraction (section 10): 14 of the 60 from the places refined on the 12 kHz audio. A and B are what the faithful per-candidate decode has (a complex baseband at 200 Hz cut to the signal's band, the 0.5 Hz / 5 ms fine search, the five LLR variants), and the cells do not.

**What it means for a Pi 3B class** (the cost claim corrected in section 13: the faithful pipeline is 0.4 s a slot on this host but 10-15 s for Deep on one Pi 3B core): the measured design that reaches `jt9`: the cells flow as a **fast candidate generator** (its search is not what fails: 50 of 60 misses have a candidate in the list) and the **faithful per-candidate decode and subtraction** for the candidates, which is the hybrid of section 1 (99.2 % of the 1 338, 475 ms a slot on this host).

## 13. Correction of the cost: the host is 25-40 times faster per instruction than a Pi 3B (2026-11-28)

Sections 1 and 12 said the faithful pipeline costs "a few seconds" on a Pi 3B because it is 0.4 s a slot on this host. **Wrong**: the host runs `qso3_busy`'s 13 050 M instructions (callgrind, `PROFILES.md` section 8) in 0.5 s, **0.04 ns an instruction**, where `PROFILES.md` reasons 1.0-1.5 ns for an A53 (and 1.0-1.6 for an A9). The ratio is 25-40, not 4-6. By the host's milliseconds a slot (mean over the 60 slots, one thread): fast 62, normal 199, deep 358 (deep + AP 392), cells SIC 3 + OSD 80; a Pi 3B core is then about **fast 1.6-2.4 s, normal 5-8 s, deep 9-14 s, the cells flow 2-3 s** (the busiest slot about 1.3 times the mean). The measured 1.07x of 4 threads leaves a Pi 3B no help from its cores for the faithful pipeline as it is. **The hybrid has to be built so that it costs less than the faithful pipeline**: the faithful per-candidate decode only for the candidates the cells flow fails (tens, not the 300-400 a pass `jt9` tries), one subtraction and one more pass at most, the candidates dealt out to the cores by the application. Its cost is not measured yet (the 475 ms of section 1 is two complete runs added).

## 14. What one candidate costs in the faithful pipeline (2026-11-28; host, one thread, `MFSK_TRACE_STAGE_FT8=1`, the 60 slots)

Mean per slot, `deep:off` 361 ms (`fast` 62, `normal` 203, `deep` + AP 398): candidate generation (spectrogram, coarse sync, FFT cache, baseline) **37 ms (10 %)**; the stages a candidate goes through (fine refine, symbol spectra, LLR, BP, OSD) **169 ms (47 %)** for 2 166 candidates refined, 614 BP and 115 OSD runs; **subtraction 113 ms (31 %)**, 56.8 calls of 2.0 ms (the `refine_dt` variant; one signal is subtracted once in each of up to three stages); the rest 42 ms.

One run of a stage, host µs (a Pi 3B about 25-40 times as much, the ratio of `PROFILES.md` section 8's instruction count to this host's time: not measured per stage): `fine_refine` 21.5, symbol spectra 2.1, `llr_fast` 4.8, `llr_partial` 26, `llr_e` 52, `llr_full` 51, **BP 100, OSD 283, subtract 1 995**.

A candidate that goes all the way (refine, spectra, the LLR variants, BP at each, OSD) costs about 0.5-1 ms on the host, **15-35 ms on a Pi 3B**; **a subtraction 50-80 ms**. So the dominant cost of a hybrid is the **subtraction** (jt9: 57 a slot; once per decoded signal: about 19, 1-1.5 s on a Pi 3B), then the cells flow's own passes (0.3-0.5 s a pass, `PROFILES.md` section 8), then the faithful decode of the candidates the cells flow fails (50 of them: 0.3-1.0 s worst case). Estimate for a Pi 3B core and one slot, the hybrid (cells search and first pass with the OSD, faithful decode of 50 failed candidates, the decoded signals refined and subtracted once at 12 kHz, one more cells pass): **about 2-3.5 s against the faithful Deep's 9-14 s**, so it takes a few seconds on one core, and an A9 core (about the same per core in `PROFILES.md`) too. **CORRECTED (section 15): that is the late path's budget, not the window's.** The decode that can still be answered has about 2 s of the slot (the signal ends at 13.14 s, the reply's audio starts at the next boundary + 0.5 s: `reference_wsjtx_tx_period_timing`); only the late path (the decode that feeds the next own period) has seconds. P1 and P2 differ in which of the two budgets a path fits. **Not measured:** the recall as a function of the number of candidates handed to the faithful decode (the public `Decoder<Ft8>` has no entry that takes a candidate list; it needs one), and the per-stage ratio to a Pi.

## 15. The hybrid measured: the faithful decode of the candidates the cells flow fails (2026-11-28; host, one thread, the 60 slots, `pi3_sweep hybrid:<N>[:<K>[:win|late]]`)

The public entry `Decoder::<Ft8>::decode_candidates` (a caller's candidate list instead of the coarse search; everything after the search is `decode()`'s: bit-for-bit the same rows for the search's own candidates, `ft8_decode_candidates.rs`) made this measurable. The cells flow decodes (`p0_late`: 30 candidates, two SIC passes, or `p0_window`), the candidates of **its first-pass search** (100, ranked by hard sync) that no decode sits on go to the faithful decode (`Depth::Deep`, AP off, `SicRounds(2)` over the given list only: **no new search on the residual**, so a signal that only shows once others are subtracted is not found), N of them, in the search's order.

| configuration | decodes of the 1 338 | extras | cells ms a slot | faithful ms a slot |
|---|---|---|---|---|
| cells `p0_window` alone (30 candidates) | 980 (73.2 %) | 32 | 22 | 0 |
| cells `p0_window` + faithful on 10 / 30 / 100 candidates | 1 047 / 1 058 / 1 061 | 52 / 58 / 60 | 22 | 28 / 46 / 71 |
| cells `p0_late` alone | 1 169 (87.4 %) | 67 | 44 | 0 |
| **`p0_late` + faithful on 10 candidates** | **1 226 (91.6 %)** | 90 | 44 | 26 |
| `p0_late` + faithful on 30 / 100 candidates | 1 232 / 1 235 (92.3 %) | 93 / 95 | 44 | 44 / 68 |
| faithful Normal, AP off (section 1) | 1 245 (93.0 %) | 86 | | 199 |
| faithful Deep, AP off (section 1) | 1 299 (97.1 %) | 115 | | 358 |

* **10 candidates give most of it**: 57 of the 66 decodes the faithful pass adds to `p0_late` come with the first 10 (a candidate costs 2.6 ms on the host here, 65-105 ms on a Pi 3B at the 25-40 ratio, **not** the 15-35 ms of section 14, which counted a candidate that goes all the way; most of these stop at the hard-sync gate).
* **It saturates at 92.3 %, 4.8 points under Deep** (**section 16: the cause named here was wrong**; it is not the candidate list but that the faithful pass decodes the unsubtracted audio): the candidate list is the cells flow's **first-pass** search. Of the 402 signals of the 22 recordings, 364 have a candidate inside the faithful refinement's reach (+-2.5 Hz, +-50 ms) in the first 100 and 375 in the first 300 (90.5 %, 93.3 %): the ceiling of a given-only list; Deep reaches the rest by searching again on its residual.
* **Cost against the faithful pipeline**: `p0_late` + 10 is 70 ms a slot (91.6 %), Normal 199 ms (93.0 %), Deep 358 ms (97.1 %): **98.5 % of Normal's decodes at 35 % of its cost**; on a Pi 3B core (25-40x) 1.7-2.8 s against Normal's 5-8 s. That is the **late path's** budget (seconds); the **window path** (about 2 s of the slot, section 13's correction) fits the cells flow alone (`p0_window` 22 ms: 0.55-0.9 s), and with 10 faithful candidates (50 ms: 1.2-2.0 s) only at the edge, on one core.
* **Extras** (decodes outside `jt9`'s list): 90 at 10 candidates against Normal's 86 and Deep's 115. Looked at one by one (no ground truth exists for these recordings, so the evidence is indirect): **67 are the cells flow's own** (`p0_late`), 23 come from the faithful pass; 60 of the 90 are also Deep's extras and 49 Normal's; of the 23 faithful-added, 18 are Deep's and 14 Normal's, **5 are in neither** (`CQ YC6RMT NJ81`, `YC6RMT IZ7NLM -22`, `<...> JH1RGG`, `JA1FWS F8BBL R-15`, `EA5INF ON2RK RR73`). **86 of the 90 carry a callsign that is a `jt9` decode in another slot** (Deep 107 of 115, Normal 81 of 86, cells 65 of 67): a false decode that yields random bits would not produce callsigns the corpus holds elsewhere. The other 4 are `SP7NHS YC2FCE -15` (attested as undecided, `ft8_real_recordings_vs_jt9.rs`) and three that Deep and/or Normal decode as well (`OK1AWC <...> +10`, `<...> JH1RGG`, `CQ SM5MNB JO89`). So no extra is specific to the hybrid in a way that suggests a phantom; this is corroboration, not proof, and the precision guard is the noise tests and the dense-grid test at `ApMode::Full` (`ft8_decode_candidates.rs`), not this count.
* **What the cells subtractor is** (the floor taken apart on a clean synthetic at its true place: -28.2 dB left, the faithful -40.3): two ideal-condition inaccuracies, the symbol-boundary position of the GFSK transition floored to 1/128 symbol (a 5 sample lag, 0.18 rad of phase after a 0-7 transition; at the sample's own distance the interior is -48 dB instead of -33) and the missing 1/8-symbol ramp of the transmitted signal at the two ends (-28.2 to -33.6 dB alone). Neither changes the real signals' depth (274 signals, -24.9 dB either way) or the 2 of 60 recovered at `jt9`'s precise places: not shipped, as they would be a further divergence from `ft8dec.c` with no measured gain.

**Not measured:** a Pi (all ratios are the 25-40x of section 13), a candidate list that includes the residual's search (the cells flow's own SIC passes find signals the first-pass list lacks: handing those over is the lever to the last 4.8 points), per-stage ratios.

## 16. Correction of section 15's ceiling: the faithful pass has to subtract what the cells flow decoded (2026-11-28; host, one thread, 60 slots, quiet machine)

Section 15 put the 92.3 % ceiling on the candidate list (no search on the residual). **Measured and refuted**: handing the faithful pass the cells flow's *second-pass* candidates (the search on its own residual, `pi3_sweep hybrid2:<N1>:<N2>`) adds 2-3 decodes (`p0_late` + 10 + 10: 1 225; + 30 + 30: 1 233). The cause is that the faithful decode of a candidate next to a signal the cells flow has already decoded works on the **unsubtracted audio**. `Decoder::<Ft8>::decode_candidates_after(&slot, &candidates, &known)` subtracts `known` (the cells flow's `DecodeResult`s) at 12 kHz with the refined place before round 0 (`DecodeRequest::known`'s path) and does not return them again.

| `p0_late` + faithful (N1 first-pass, N2 second-pass candidates; `SicRounds(2)`) | decodes of the 1 338 | ms a slot (cells + faithful) |
|---|---|---|
| cells alone | 1 169 (87.4 %) | 46 + 0 |
| 10 + 0, no `known` | 1 223 (91.4 %) | 46 + 18 |
| 10 + 10, the 4 / 8 / 12 strongest of `known` | 1 235 / 1 242 / 1 249 | 46 + 41 / 52 / 66 |
| 10 + 10, all of `known` (about 20 a slot) | 1 261 (94.2 %) | 46 + 96 |
| 30 + 30, the 12 strongest | 1 262 | 45 + 85 |
| **30 + 30, all of `known`** | **1 277 (95.4 %)** | 45 + 114 = **159** |
| 100 + 100, all of `known` | 1 281 (95.7 %) | |
| faithful Normal, Deep (section 1) | 1 245 (93.0 %), 1 299 (97.1 %) | 199, 358 |

* **Subtract only what is near a candidate** (`known` filtered to the signals within P Hz of a candidate that is sent, `hybrid2:..:near<P>`; quiet machine, ms a slot as cells + faithful):

| | decodes | ms a slot | signals subtracted a slot |
|---|---|---|---|
| 10 + 0, no `known` | 1 223 | 44 + 18 = 61 | 0 |
| 10 + 10, within 10 Hz | 1 246 | 45 + 41 = 86 | 4.9 |
| **10 + 10, within 30 Hz** | **1 256 (93.9 %)** | 45 + 55 = **100** | 8.9 |
| 10 + 10, all | 1 261 | 45 + 96 = 141 | 20.6 |
| **30 + 30, within 30 Hz** | **1 275 (95.3 %)** | 44 + 97 = **141** | 15.8 |
| 30 + 30, all | 1 277 (95.4 %) | 44 + 113 = 157 | 20.6 |

  Within 30 Hz loses 2-5 decodes and saves 21-57 % of the subtractions; **10 + 10 within 30 Hz (100 ms, 1 256) beats Normal (197 ms, 1 245) at half the time**, and at the same 141 ms 30 + 30 within 30 Hz (1 275) beats 10 + 10 with all (1 261): spend the time on candidates, not on subtracting far signals. The frontier (ms a slot: decodes): 46: 1 169, 61: 1 223, 86: 1 246, 100: 1 256, 141: 1 275, 157: 1 277, Deep 360: 1 299.
* **A subtracted signal costs 4.4 ms on the host** (the faithful pass's time minus the same without `known`, over the 1 176 signals handed over) and each is worth about 1.1 decodes at 30 + 30: that is the knob. All of `known`: **98.3 % of Deep's decodes at 44 % of its time, and 103 % of Normal's at 80 %**.
* On a Pi 3B core at the 25-40x of section 13: 30 + 30 with all of `known` 4.0-6.4 s, 10 + 0 without `known` 1.6-2.6 s (91.4 %). The late path's budget (seconds), not the window's (about 2 s): the window keeps the cells flow alone, or 10 + 0 at its edge.
* **Extras**: 129 at 30 + 30 with all of `known` (Deep 115); 122 of the 129 (94.6 %) carry a callsign that is a `jt9` decode in another slot (Deep 107 of 115, 93 %). 83 are also Deep's. Same indirect evidence as section 15, not proof; the precision guard is `ft8_decode_candidates.rs` (noise candidates, a dense grid at `ApMode::Full` with `known`).
* The timings of an earlier run of this section were taken while `scripts/verify.sh` ran and are discarded; these are from a quiet machine.

**Not measured:** a Pi; other choices of which of `known` to subtract (the SNR order and the 30 Hz window are tried; the signals whose skirts overlap a candidate in time as well, or by the candidate's own hard-sync loss, are not); the cells flow's own subtraction left in (`known` is subtracted by the faithful subtractor again from the original audio, so the cells flow's residual is not used).

## 17. The light subtraction: the place is not what limits it, and a finer amplitude makes it deeper and worse (2026-11-28; host, 60 slots / the 22 recordings)

Section 15 left the cells subtractor's 1.5-4 dB on real signals unexplained, with the place of the estimate the suspect. **Measured against `jt9`'s precise places** (286 decoded signals matched by message, `ft8dec_sic.rs` measurement `place_error_and_depth_at_jt9s_place`, `PI3_SCRATCH`): the cells estimate is off by **0.19 Hz and 2.9 ms (median; 90th percentile 0.45 Hz, 9.4 ms)**, not the 28 ms / 0.4 Hz said earlier, with a bias of -2.5 ms in DT. Given `jt9`'s places the cells subtraction is **not deeper**: median left -25.2 dB at its own place and -21.3 dB at `jt9`'s on the strong signals (+8 dB and up), -20.7 / -19.2 at 0..8 dB, and the earlier 2 of 60 recovered stays 2. The place is not the limit; `ft8dec_orig.c` and the port are the same line by line (section 15), so the limit is the model.

**The model's resolution**: `ft8dec` estimates one complex amplitude per symbol and interpolates linearly between symbol centres. Splitting each symbol into R correlation segments (the partial sums of the correlation the loop already computes; the amplitude interpolated between the segment centres; R = 1 reproduces the current residual exactly):

| R | own-tone energy left, +8 dB and up (mean) | `p0_late` decodes of the 1 338 | `sic3osd` |
|---|---|---|---|
| 1 (`ft8dec`) | -24.9 dB | **1 169** | **1 190** |
| 2 | -29.3 dB (the faithful: -26.7) | 1 154 | 1 176 |
| 4 | -31.9 dB | 1 099 | 1 114 |

The subtraction gets **deeper on the signal's own tones and finds fewer signals**: the neighbours are lost, and neighbours of a strong signal are what a SIC is for. Hypothesis, not tested here: a correlation over a segment shorter than a symbol is not orthogonal to the tones 6.25 Hz away, so the amplitude absorbs a neighbour's energy and subtracts it too; the per-symbol correlation's orthogonality is the design, and the metric "the signal's own tone energy left" (section 15's table and `depth_against_the_faithful_subtraction`) cannot see that damage. A depth that counts should be measured on the neighbouring tones' energy, or, as here, on the decodes. **Not changed**; the experiment is not kept in the tree (`seg_amplitude_experiment.patch` in the study's scratch).

Two more cheap variants on the same ruler (`p0_late` decodes of the 1 338; the baseline 1 169):

| variant | decodes | depth, +8 dB and up |
|---|---|---|
| amplitude sampled every half symbol from a window of **one symbol** (two segments summed, so the orthogonality to the neighbouring tones of a symbol-long window is kept) | 1 162 (`sic3osd` 1 184 against 1 190) | -25.7 dB (against -24.9) |
| the subtracted amplitude scaled by 0.9 / 0.95 / **1.0** / 1.05 | 1 111 / 1 139 / **1 169** / 1 156 | |

Neither helps: a denser sampling of a symbol-long window changes the depth by 0.8 dB and the decodes by -7, and **the unbiased amplitude (gain 1.0) is the optimum** of the scale (a smaller one leaves the signal, a larger one subtracts the neighbour). So `ft8dec`'s per-symbol amplitude with linear interpolation is at a local optimum for these knobs, on this corpus: the place, the amplitude resolution, the sampling and the scale have each been moved and none improves the decodes. What is left of the 1.5-4 dB to the faithful subtractor is its different construction (a continuous low-pass of the whole-slot correlation, 12 kHz, the refined place); it is not reached by a cheap change of this one. The two experiments are not in the tree (`seg_sliding_gain_experiment.patch` in the study's scratch).

## 18. The window path: what the cells flow can do in the time after the signal (2026-11-28; host, one thread, the 60 slots, `pi3_sweep cells:o:c<M>,k<K>,p<P>`)

The reply that can still be sent has about 1.2-2 s after the signal's audio is in (section 15). The cells flow's search and demux are streamed while the slot is received (the search and the cells of 30 candidates cost about **12 ms a slot** on the host, `cands:30` including the wav read and the resample: not in the window); what is in the window is the candidate rounds and, with the subtraction, the second pass (a read of the slot through `ResidualSource`, a new search, a second round). `p0_window`'s configuration with M candidates (also the search's size), the OSD capped at K (0 = off) and P passes:

| P | K | M = 10 | 15 | 20 | 30 | 50 |
|---|---|---|---|---|---|---|
| 1 | 0 | 527 / 13 | 741 / 14 | 876 / 16 | 942 / 22 | 944 / 30 |
| 1 | 3 | 533 / 14 | 755 / 15 | 902 / 19 | 975 / 27 | 977 / 42 |
| 1 | 6 | 534 / 13 | 756 / 15 | 906 / 19 | **980 / 28** | 982 / 43 |
| 2 | 0 | 663 / 24 | 892 / 27 | 1 029 / 31 | 1 111 / 40 | 1 115 / 53 |
| 2 | 3 | 684 / 27 | 924 / 30 | 1 077 / 37 | 1 158 / 49 | 1 162 / 71 |
| 2 | 6 | 687 / 28 | 927 / 31 | 1 085 / 38 | **1 169 / 52** | 1 173 / 72 |

(decodes of the 1 338 / ms a slot, all of it, the streamed part included.)

* **30 candidates is the knee**: 50 adds 2-4 decodes for 15-20 ms; below 30 the decodes fall fast (20: -74 at P = 1, -84 at P = 2).
* **The OSD cap**: K = 6 against K = 3 is +5 / +11 decodes for 1-3 ms; no OSD costs 38-58 decodes for 5-7 ms. Keep 6 (the CoreS3's value).
* **The subtraction pass (P = 2)** is the lever: +189 decodes for +24 ms at M = 30, +179 for +19 at M = 20.
* **The frontier** (ms a slot: decodes): 13: 534 (M 10) - 15: 756 (15) - 19: 906 (20) - **28: 980 (30, P 1)** - 38: 1 085 (20, P 2) - **52: 1 169 (30, P 2)**. M = 15 with P = 2 (31: 927) is beaten by M = 30 with P = 1 (28: 980).
* **What is in the window**, the part after the streamed 12 ms (host, then a Pi 3B core at 25-40x, an upper bound: the streamed part is not all hidden): P = 1, M = 30: about 16 ms, **0.4-0.65 s**, 980 decodes (73 %); P = 2, M = 20: about 26 ms, **0.65-1.0 s**, 1 085 (81 %); P = 2, M = 30: about 40 ms, **1.0-1.6 s**, 1 169 (87 %): inside the 1.2-2 s at the lower ratio and at the edge of it at the upper one. The late path adds the faithful pass on top (section 16).
* **Not measured:** the Pi, and the split of a pass between its streamed and its post-signal part on the board (the host's 12 ms is the search alone; the demux of the cells is in it, the second pass's read is not).

## 19. The CoreS3 measured: the window path and the late path against the host, one slot (2026-11-28; `embedded-poc/m5stack-cores3-app`, `ft8-cells-flow`, wav_sim `qso3_busy`, 6 slots, `logs/cells_window_late_2026-11-28.log` not committed)

`MFSK_CORES3_FORCE_MODE=decode cargo build --release --features ft8-cells-flow`, `scripts/capture.sh ... 150 "LATE slot"` (the library of this tree: `decode_candidates` and the rest of this study are not in the app). Board (ESP32-S3 at 240 MHz, PSRAM) against the host of sections 13-18 (one thread), the same slot:

| | board | host | board / host |
|---|---|---|---|
| window path (`p0_window`, 30 candidates, P = 1): decodes | 11 (9 in hand at the emit) | 11 | |
| the work of the slot's jobs: search + decode + OSD pass + SNR/publish | 450 + 617 + 209 + about 65 = **about 1 340 ms** | 36-38 ms (the whole call) | **about 36x** |
| **after the slot's end** until the list is out | **383-386 ms** (6 of 6 slots) | | |
| emit (row pair 87) | 2 016-2 023 ms before the end | | |
| late path (`p0_late`, core 1): decodes | 13 (2 new) | 13 | |
| the late path's SIC job | **3 781-3 804 ms** | 58-76 ms for the whole `p0_late`, about 50-60 ms without the first search | **about 63-80x** |

* No reset, 0 samples dropped in the 6 slots (the `USB (11)` reset reason is the flash); waterfall FFT 2.5-3.3 ms a row while the SIC runs on core 1.
* **The ratio is not one number.** The window path's work is 36x the host's: it streams (the search's 8 ranges and the demux are done while the slot comes in) and runs on one core. The SIC job is 63-80x: on core 1, its working set in PSRAM, the search's FFTs and the cells' copy; section `ESP32S3_TWO_CORE_BEHAVIOUR.md`'s serialized PSRAM misses are the measured reason such a job costs more than its instruction count. Section 13's 25-40x was an A53 / A9 core's instruction time; **for this board the SIC is 2x worse than that range, the window path within it**.
* **What the board says about section 18**: the window path of the app is `p0_window` (P = 1), and it is out 383-386 ms after the end with 2 s of the slot's work done before it. Section 18's P = 2 in the window (a second pass, +24 ms on the host) is, on the board, **a 3.8 s job on core 1: not a window job**; with 20 candidates it would be about 2.5 s. P = 2 in the window is a fact of a faster core than the ESP32-S3's (the Pi 3B's A53 at 25-40x of the host: 1.0-1.6 s from section 18, from the instruction ratio, not measured), not of this board.
* **Not measured:** the Pi; the faithful pass (it is not in the app); a radio, WiFi and the USB host running.

## 20. The faithful profiles P1 / P2: what AP does, and the floors (2026-11-28; host, one thread, the 60 slots and the 22 recordings)

`mfsk_core::decoder::Profile` (P1 = `Depth::Normal`, P2 = `Depth::Deep`, AP on, lanes 2 / 4, slot budget 0.8 / 0.9 of the period) for `skimmer --profile` and `mfsk-serve --profile`. The choice of Normal and Deep follows from the code's own comments: FT8's early decode (rows at ~11.8 s, `skimmer`'s `early_decode`) needs the `SicEarly` checkpoints that only Normal and Deep have, and `--slot-budget` (`PeriodBudget::share`) stops a decode that does not fit its period and reports what it has, which is how a heavy depth runs on a slow core. The Pi 3B figures are the host's times x 25-40 (section 13): Normal 5-8 s, Deep 9-14 s a slot on one core, an A9 Normal 7-12 s (`PROFILES.md` section 8); **no decode was run on either**, and the shares 0.8 / 0.9 are guesses.

AP, on the 60 slots (1 338 of `jt9`'s; ms a slot, host):

| | decodes | extras | ms |
|---|---|---|---|
| Normal, AP off | 1 245 (93.0 %) | 86 | 200 |
| Normal, AP `cq` = `full` | 1 258 (94.0 %) | 98 | 208 |
| Deep, AP off | 1 299 (97.1 %) | 115 | 353 |
| Deep, AP `cq` = `full` | 1 308 (97.8 %) | 132 | 391 |

`cq` and `full` are the same here: with no station and no QSO context the full AP has only the CQ hypothesis. The profiles use `CqOnly` (no QSO assumed). AP adds 13 (Normal) and 9 (Deep) decodes and 12 and 17 extras; 12 of the 13 and 15 of the 17 carry a callsign `jt9` decodes in another slot; the others are `CQ SP3OWE JO81` (a callsign seen nowhere else in the corpus: the blind-CQ AP is the phantom-prone path) and `CQ YC2FCE OI42` (the callsign is in the neighbouring slot's attested extra).

Tier B (`tests/profile_conformance.rs`, the 22 recordings, `jt9`'s 402): **P2 finds 402 of 402** with no unattested extra (the same as `ft8_real_recordings_vs_jt9.rs`); **P1 finds 387 of 402** (96.3 %), no unattested extra, floor 385. With P1 at `Depth::Fast` it finds 326 (the mutation that shows the floor can fail).

## 21. Correction: the cells flow of sections 15-19 ran the blind-CQ AP rung the board does not have (2026-11-28; host, one thread, quiet machine, `hosttest/icap-app` `icap-p0`)

`pi3_sweep` is built with `full`, which has `fft-rustfft`, and `process_candidates.rs` step 4 (the AP rung: the blind-CQ pass on every candidate that failed BP and OSD) is `#[cfg(feature = "fft-rustfft")]`. **So every cells-flow figure of sections 15-19 included an AP pass the board (`fft-extern` only) does not run.** `icap-p0` (new: `CellsOptions::p0_window()` / `p0_late()` over the same 60 slots, resampled by the crate's own `resample`, in the board's build) measures the cells flow as the board runs it:

| 60 slots, host, one thread | decodes of the 1 338 | extras | ms a slot |
|---|---|---|---|
| `p0_window`, no AP (`icap-p0`) | 985 | 34 | **9.7** |
| `p0_window`, blind-CQ AP (`pi3_sweep`) | 980 | 32 | 28.1 |
| `p0_late`, no AP (`icap-p0`) | 1 171 | 62 | **24.8** |
| `p0_late`, blind-CQ AP (`pi3_sweep`) | 1 169 | 67 | 51.5 |

(the resampler differs, so the decodes are not compared to the unit; `pi3_sweep`'s ms include its resample, about 3-5 ms; `qso3_busy`: 11 ms and 23 ms without AP, 35 ms and 55 ms with.) **The AP rung is 2-3x the time of the whole call and adds no decode that shows** (`S3_CELLS_IMPLEMENTATION_PLAN.md` measured 1 531 M against 924 M instructions for +6 hits and +8 extras).

What changes:
* **Section 18's ms** (the window path's table) were 2-3x too high for the board's flow; the decodes stand. The window path (P = 1, M = 30) is 9.7 ms, **0.24-0.39 s on a Pi 3B core** (25-40x); `p0_late` (P = 2) 24.8 ms, **0.62-1.0 s**, which fits the 1.2-2 s of the window, not the 1.0-1.6 s section 18 said.
* **Section 19's ratios** were against a host figure with the AP rung. Against the board's own flow: the window path's jobs (about 1 340 ms) are **about 120x** the host's 11 ms; the SIC job (3.79 s) is **about 165-200x** the host's 19-23 ms (`p0_late` less the first search). The ESP32-S3 is slower than section 13's "25-40x" by 3-5x, not 1-2x.
* The hybrid's cells half (sections 15-17) used `pi3_sweep`'s cells; its ms are 15-26 ms a slot too high (cells 46 ms a slot is about 25 ms); its recall stands.

## 22. The QSO-context AP at the reply window, measured on the host (2026-11-28; `mfsk-core/examples/ap_qso.rs`, AWGN band, 10 trials a cell, one thread)

A synthetic reply to the operator (`K1JT HA0DU` + `RR73` / `-15` / `KN07`, at 1 500 Hz, 0.5 s start) in AWGN, SNR in 2 500 Hz, decoded by: `cells` (`p0_late`, as the board runs it: no AP for a reply), `blind` (faithful Deep, AP off), `ap` (faithful Deep with the QSO context and the Rx frequency: its own search), `c1` / `c9` (`decode_candidates` of **only the partner's frequency**, one candidate 1 Hz and 20 ms off, or a 3 x 3 grid of 3.125 Hz and 40 ms, with the QSO-context AP). Hits of 10; the 50 % crossing is interpolated, +-1 dB.

**The AP passes alone** (`AP_QSO_NOGRID=1`: no grid for the partner, so the a8 list decoder, which needs both calls, a grid and the Rx frequency, does not run):

| reply (QSO progress) | cells | blind Deep | AP passes at the partner (`c1` / `c9`) | gain over blind |
|---|---|---|---|---|
| `RR73` (Rogers: passes 8-11) | -17.7 dB | -20.0 | **-22.6** (10 / 10 at -20; 8 / 10 at -22) | +2.6 dB |
| `-15` (Replying: 5, 8) | -18.4 | -19.8 | -21.8 (10 at -20; 4 / 5 at -22) | +2.0 |
| `KN07` (Calling: 5, 12) | -18.7 | -20.5 | -20.8 (7 / 8 at -20; 0 at -22) | +0.3 |

* **The cost at the partner's frequency is almost nothing**: `c1` 0.3-3.2 ms and `c9` 0.8-25 ms **a slot on the host** (**10-130 ms on a Pi 3B core, 25-40x**), against 80-110 ms for the faithful decode of the whole slot. The same hits as the whole-slot AP (`ap`, no grid): the partner's candidate is all the AP needs.
* **The cells flow's window threshold for a reply is about -18 dB**; the AP passes at the partner's frequency take it to **-20.8 to -22.6 dB**: **+3 to +5 dB** on the replies QSO is made of.

**With the a8 list decoder as well** (the grid given: `ap`, `c1`, `c9`): the 50 % crossing is about **-23.2 to -23.4 dB for all three replies** (10 / 10 at -22; 1-3 / 10 at -24): another **+1 to +2.6 dB** over the AP passes (`KN07` +2.5: 0 / 10 against 10 / 10 at -22 dB). But a8 costs **about 110 ms a slot on the host** (it works on the 192 000-point FFT cache): **2.8-4.4 s on a Pi 3B core**, 13 s on the ESP32-S3: not a window job as it is.

**Phantoms** (the band holds nothing but the reply): over 3 scenarios x 6 SNRs x 10 trials x 5 decoders (900 decodes) **4 rows that are not the reply**: `CQ XW6GWL GC94` from `cells` at -16 dB (this build has the blind-CQ rung); `K1JT HA0DU NQ35` from `ap` at -24 dB (**both calls locked: a message that looks like the partner's** and is false); `RK4NCK 0I8GOJ 539 0450` from `blind` and `ap` at -22 dB (a free-text-like decode without any AP). **300 noise-only slots with the context set** (`c9`, 100 for each progress): 0. The rate of the 61-bit lock's false accept is not bounded by this size of test (`ft8_ap_osd_false_accept.rs` measures it per CRC-valid draw).

**What it says for the QSO window**: the AP at the partner's frequency is worth +3-5 dB over what the window decodes now, for a few ms of the host's time (the cells' side of the work is the same BP and OSD with a mask, shared code); the a8 list decoder is worth up to 2.6 dB more but is a different, heavier mechanism. **Not measured:** a busy band (the replies are alone in AWGN), the cells flow's own AP (not written), the board, fading, a larger noise test for the 61-bit lock.

## 23. The QSO-context AP: a busy band, fading, and the false accepts of the locks (2026-11-28; `ap_qso busy | fading | phantoms`, host, one thread; AP passes alone, `AP_QSO_NOGRID=1`, so no a8)

**A busy band** (the reply added into 5 real recordings, `qso3_busy`, `websdr_test7/5/11`, `191111_110630`, at the recording's free frequency, 2 start times: 10 trials a cell; the level on the decoder's own SNR scale from a calibration probe, so these dB are not section 22's AWGN ones). Hits of 10 at -22 / -24 dB; `c1o` is `c1` with the AP off (the location alone):

| reply | cells | blind Deep | `c1` (location + AP) | `c1o` (location only) |
|---|---|---|---|---|
| `RR73` | 5 / 3 | 6 / 2 | **10 / 9** | 7 / 4 |
| `-15` | 7 / 4 | 6 / 1 | **10 / 7** | 7 / 7 |
| `KN07` | 7 / 6 | 7 / 3 | **9 / 6** | 6 / 4 |

In a busy band the cells flow and the faithful whole-slot decode fail together at about -22 dB (the candidate search): **knowing where the partner is adds about 1 dB, the AP another 1.5-2 dB**, 2.5-3 dB over the window's decode in all; `c1` costs 0.3-2 ms (`c9` 0.7-15 ms) a slot.

**Fading** (flat Rayleigh, about 1 Hz Doppler, mean SNR, AWGN band, 20 trials; the 50 % crossing interpolated, +-1 dB):

| reply | cells | blind Deep | `ap` whole slot | gain over blind |
|---|---|---|---|---|
| `RR73` | -16.6 | -18.6 | **-22.0** | +3.4 dB |
| `-15` | -16.7 | -18.6 | -21.0 | +2.4 |
| `KN07` | -17.0 | -18.6 | -19.6 | +1.0 |

(`c1` / `c9` equal `ap`: 20 / 20 at -20 for `RR73`; the cells flow loses 2 dB to fading more than the faithful decode does.)

**The false accepts** (the part that matters in a QSO). **Noise-only slots, the context set, the partner's frequency given (`c9`: 9 candidates), 2 000 slots for each progress: Rogers 2, Replying 1, Calling 0 decodes**: `K1JT HA0DU RR73` (**a false RR73 from the partner**), `K1JT HA0DU R FA89` (twice). That is about **5 x 10^-4 a slot** (3 in 6 000), about 10^-3 a slot in the Rogers stage: a QSO machine that takes an `RR73` as its end would log a QSO that did not happen about once in a thousand slots of waiting. **Real recordings with calls nobody in them has** (22 recordings x 3 progress x 5 Rx frequencies, 15 candidates each, 390 decodes): **0** with the locked calls. The faithful pipeline's own guards (`ap_max_errors`, the locked calls reappearing in the message) are what let these through; WSJT-X's AP has the same property. A design that acts on an AP decode needs a stricter acceptance for the 61- and 77-bit locks (fewer hard errors, a DT and frequency consistent with the partner's, or a second sighting) and a noise test sized for it.

**Phantoms outside the context** in the fading run: 5 rows in 1 800 decodes (blind decodes `H99JNS LP4ZVB PO38`, `SF4IKM RR73; 4P3UMO <...> -28`, a `CQ N5MAX AO31` of the AP decode): the faithful pipeline's own, as in section 22.

## 24. The cells flow's own AP rung (2026-11-28; `ApContext`, `CellsWorker::with_ap_context` / `decode_partner`, `tests/cells_ap.rs`, `ap_qso cellsap`)

The AP of section 22 for the board's flow (no `fft-rustfft`: step 4 of the faithful ladder is not there). **A QSO context** (`ApContext`: the operator's and the partner's call as `pack28` words, the stage, the partner's frequency and DT; `Copy`, 20 bytes, no `String`) sets on a `CellsWorker` the AP rung of `process_one_candidate_inner`: for the stage's passes (Replying: 8, 5; Rogers: 9, 10, 11, 8; Calling: none) the pass's bits locked to `+-1.1 max|llr|` in the one-symbol LLRs `a` and `d`, the masked NMS BP, and with the OSD the masked OSD of the BP sum after one and two iterations. The locks are the faithful hypotheses bit for bit (`ap_context::tests`); BP and OSD are the shared code with the mask they already took. `decode_partner` searches the store `+-25 Hz` round the partner (12 candidates, the DT window applied, 4 decoded, no minimum score, no pre-BP gate). A divergence from WSJT-X, off unless a context is set.

**AWGN, the cells window** (`p0_window`'s staircase and the capped OSD; 100 seeds a cell; hits without the context -> with it; the partner at +1 Hz, 20 ms):

| reply | -18 dB | -20 | -22 | -24 | 50 % crossing |
|---|---|---|---|---|---|
| `RR73` | 74 -> **100** | 2 -> **94** | 0 -> **41** | 0 -> 4 | -17.7 -> **-22.3 dB** (+4.6) |
| `-15` | 83 -> **100** | 3 -> **79** | 0 -> 10 | 0 -> 0 | -18.3 -> **-20.9 dB** (+2.6) |
| `KN07` (Calling: no pass) | 88 -> 88 | 2 -> 2 | 0 | 0 | unchanged |

The rung costs **+2-5 ms a slot on the host** (the partner's candidates and, on a failure, the masked BP and OSD of 1-4 passes on the few candidates near the partner). It is about 1 dB under the faithful pipeline's AP (`c1`, section 22: -22.6 / -21.8), which tries four LLR variants and the full OSD depth.

**The hard-error ceiling does not separate the false accepts of a full lock, and a tighter one was dropped.** A ceiling of 32 for the locks of 70 bits or more (`RRR`, `RR73`, `73`: only the 97 parity bits are free, so noise gives 48.5 +- 4.9 errors against the locked LLRs and 36 is 2.5 sigma) was first built, and its evidence ("the rung's decodes had at most 31 errors") was circular: the rung had rejected 32 and over. With the ceiling lifted (200 seeds, `RR73`): at -20 dB 3 of 196 true decodes have 32 or more errors, at -22 dB **7 of 87 (8 %, up to 35)**; and the one false `RR73` of the faithful pipeline (section 23) has 24. A ceiling of 32 would lose 8 % of the true decodes at the edge for no measured gain, so the generic `DecodeStrictness::ap_max_errors` stays; what keeps the false accepts down is the window. (The argument for 2.5 sigma holds for the BP path only: the OSD counts against the BP sum with the locks held, which has pulled the unlocked bits toward the one codeword, so its noise errors are fewer than Binomial(97, 0.5).)

**The false accepts, and what the DT window does** (noise-only slots with the context set, 6 000 for each stage): without the window (frequency only, `ApContext::WINDOW_HZ` = 50) the rung let through **3 (Rogers) and 4 (Replying) messages with the locked calls** (`K1JT HA0DU R EC96`, `K1JT HA0DU MN51`, `K1JT AA8YRJ IO35` ...), with hard errors 24-32 that the replies' own errors (up to 31) cannot be told from. A lock does not care where a candidate sits, so what a false accept scales with is the window the rung may run in: **a DT window of +-0.4 s** (`ApContext::WINDOW_DT_S`; the partner's clock does not move between two periods, and the application knows its last DT) gives **0 and 0 in 6 000 slots each**. The faithful pipeline's AP on the same noise with a 9-candidate grid (no DT window, `decode_candidates`): Rogers 3 (`K1JT HA0DU RR73` with 24 errors among them), Replying 2, 0 of 390 in real recordings with other people's calls. The candidates an application gives it are its window; one (`c1`) is a tenth of the grid's.

**The build of these measurements.** `ap_qso cellsap` and `mfsk-core/tests/cells_ap.rs` build with `full`, so `fft-rustfft` is on and the faithful ladder's blind-CQ pass (step 4) runs on every failed candidate **before** the rung (section 21's trap again: the board has no such pass). `hosttest/icap-app/tests/icap_ap.rs` runs the rung in the board's build (`fft-extern`, no `fft-rustfft`): `RR73` at -20 dB, 12 seeds, **12 of 12 with the context, 0 of 12 without**; 100 noise slots, no locked message; the partner 200 Hz or 1.5 s away: not decoded. The crossings above are from the `full` build; the board build's are not tabulated.

**Queues** (a review finding, fixed): in `OsdMode::Defer` a candidate that the rung decoded still had its blind OSD attempt queued (taking one of the slot's `k` places and, on a kept worker, running against the next slot's cells), and `decode_partner`'s candidates queued theirs: the first is now dequeued when the rung accepts, the second truncated after `decode_partner` (`tests/cells_ap.rs`, both mutation-checked).

**Not done:** the application (the partner's frequency and DT have to reach `HeardItem` and the `Activator`, `ApContext::new` at the reply deadline), the board (cost and memory of the rung: estimated 25-100 ms a candidate; stack of `ap_rung_cells` estimated +0.5-2 KB, `#[inline(never)]`), a larger noise test for the 61- and 77-bit locks (0 in 12 000 is a bound near 2.5e-4 a slot at 95 %), fading and a busy band for the cells rung (section 23 measured the faithful one), the narrow-band a8.

## 25. The cells flow's AP rung on the CoreS3 (2026-11-28; `ft8-ap-bench`, `embedded-shared/src/apps/ap_bench.rs`, logs `embedded-poc/m5stack-cores3-app/logs/ft8-ap-bench_{third,fourth}_2026-11-28.log`, not committed)

One core, no WiFi / USB host, the slots made on the board (AWGN, the reply `K1JT HA0DU RR73` or `-15` at 1 500 Hz, 0.5 s in, 5 seeds a cell), the search once a slot and the window round (`p0_window`'s staircase, the capped OSD K = 6) run without and with the context, then `decode_partner`. The predictions were written in the module doc before the first run.

| reply, SNR | hits without -> with the context | the round without | the context adds (round + partner - round without), median (max) | `decode_partner` alone, median (max) |
|---|---|---|---|---|
| `RR73`, -20 dB | 0 -> **4** of 5 | 534 ms | **182 ms** (658) | 110 ms (372) |
| `RR73`, -22 dB | 0 -> 2 | 587 ms | 478 ms (894) | 271 ms (441) |
| `-15`, -20 dB | 1 -> **5** | 496 ms | **49 ms** (68) | 40 ms (46) |
| `-15`, -22 dB | 0 -> 2 | 566 ms | 222 ms (245) | 123 ms (134) |

* **Against the predictions** (written first; the stop rule was 25 %): a reply that decodes **20-150 ms** for `decode_partner`: 40 ms (report) and 110 ms (`RR73`), inside; the worst case, the partner's four candidates failing every pass: **0.4-0.9 s for Rogers** (observed 0.27 s median, up to 0.44 s alone and **0.89 s added** in a slot) and **0.2-0.45 s for Replying** (observed 0.12-0.13 s alone, **0.22-0.25 s added**): inside, at the low end for Replying. The host's hit counts (12 of 12 at -20 dB in section 24) are not reproduced 5 seeds at a time (4 of 5): the generator differs (a sum of twelve uniforms for the board's Gaussian: no double-precision `ln` / `cos` on the LX7, which cost 14 s a slot and tripped the task watchdog in the first run).
* **The -27 dB runs were not a worst case**: the partner's search finds no candidate in the DT window (0 BP calls, 10 ms), so the failing cost above is read from -22 dB.
* **Stack**: after the window round without the context 25 036 B free of the 48 KB bench thread (the staircase and the OSD use 15.9 KB of the 40.9 KB there was), after the context's round and `decode_partner` **23 132 B**: **the rung adds 1.9 KB** (predicted 0.5-2). In the app's decode task (40 KB, 10.8 KB free at its lowest, section 34) that leaves about 8.9 KB. Heap: the minimum free internal DRAM 244 KB (boot 258 KB), PSRAM 6.1 MB.
* **Self-check**: hits at -20 dB 9 with against 1 without, no message with the locked calls in 8 noise slots (`ap-bench: done ok`). Two runs agree (the context adds 182.1 and 181.9 ms, the median over 20 slots).

**What it means for the window** (list out 380-390 ms after the slot's end, the reply's deadline 500 ms: about 120 ms of slack on core 0): the context costs 50-180 ms in the slots where the reply decodes and up to 0.9 s in the slots where the partner's candidates fail. **On core 0, in the window path, it does not fit**. `decode_partner` alone is 40-440 ms; on **core 1**, started at the emit (0.81 s before the boundary: about 1.3 s to the deadline) with the two-core factor of 1.5, 60-660 ms: that fits, and the round's own +0.05-0.27 s (the rung inside `decode_round_osd`) is avoided by setting the context on core 1's worker only. **Not measured:** both cores running, the app (WiFi, the panel, the real emit time of a slot whose reply is complete), a busy band at a calibrated level (the busy rows are at a nominal amplitude), the partner's frequency and DT reaching the worker.

## 26. A noncoherent list decision on the cells (B1, 2026-11-29; `ap_qso a8cells`, host, research: no library change)

The question (section 25's plan): how much of a8's +1-2.6 dB over the AP passes survives on the cells, which keep amplitude and no phase. The experiment, `cargo run --release --example ap_qso --features full,fixed-point,internal-testing -- a8cells 20 2000 4000`: the stage's hypotheses (Rogers: `RRR`, `RR73`, `73`; Replying: the reports -30..+32 with and without `R`, 126) as tone sequences, scored on the cells around the partner (+-3 bins, +-5 rows of the AP context's frequency and DT, **both known exactly in the test**) by S1 = the sum over the 79 symbols of the expected tone's share of the symbol's energy, S2 = the sum of `ln(1 + 7 e / (rest))`; the best place and hypothesis. The threshold is the largest score of 2 000 noise slots (the cells are not a8's iid exponentials: measured, not derived). AWGN, 20 seeds a cell, the AP passes and the blind window on the same slots:

| SNR | Rogers `RR73`: blind / AP passes / list S1 / S2 | report `-15`: blind / AP / S1 / S2 |
|---|---|---|
| -20 dB | 0 / 19 / 20 / 20 | 0 / 15 / 20 / 20 |
| -22 dB | 0 / 6 / 20 / 20 | 0 / 1 / 20 / 20 |
| -24 dB | 0 / 0 / 19 / 18 | 0 / 0 / 15 / 16 |
| -26 dB | 0 / 0 / 6 / 6 | 0 / 0 / 3 / 3 |

* **Gain**: the list's 50 % point is about -25.5 dB (Rogers) and -24.5 dB (report) against about -22 dB (Rogers) and -20.5 dB (report) for the AP passes: **+3.5-4 dB**, more than the +1-2.6 dB a8 is worth over the AP passes on the host (section 22): the AP passes decode through BP/OSD and need the codeword to pass the CRC and the hard-error ceiling, the list decision needs only the best of a few hypotheses to stand out. This is a **different decision** (a hypothesis picked from a list, no CRC, no 77 bits decoded): what it accepts is "the partner sent one of these", and the false accepts are the cost.
* **False accepts, the gate's other half, not met**: with the threshold at the largest of 2 000 noise slots, 4 000 other noise slots gave **Rogers 1 (S1) / 0 (S2), report 6 (S1) / 4 (S2)**: about 1e-3 a slot for the report list of 126, a false report or `RR73` about once in a thousand slots (4 hours of slots), the order of section 24's lock false accepts (5e-4) before the DT window. The gate asked for 0 in 12 000 slots: it needs a threshold from the tail (more slots, or a margin), and what that margin costs in the hits is **not measured here** (the curve above falls 4 dB from all hits to a quarter, so a margin of 0.5-1 dB is cheap, but that is read, not measured).
* **Not measured**: the place the partner is at (exact here; the app's `HeardItem` frequency and DT from the previous slot, an off-grid half-bin, a drifting clock), the busy band and fading (the AP rows of sections 23-24 say what those do to the AP; the list's behaviour there is unknown), the cost on the board (the experiment makes 77 places x 129 hypotheses x 79 symbols: about 0.8 M lookups, 5-10 ms on the host: ~0.6-1.2 s on the LX7 as written, **too slow unchanged**; narrowing the place set to the context's (a few) and the Replying list to the reports around the partner's last report would bring it to about 1 ms, again an estimate), the message that follows a hit (the list decision names a hypothesis, the QSO code then needs the message and its 77 bits, which it can pack from the hypothesis).

**Decision for B2** (the plan says: only with a new approval): the gain is large enough to justify the product step, the false-accept budget and the cost on the board are the two things B2 must settle first. Nothing here is wired into the library.

### 26.1 The speed (2026-11-29; `A8_FAST=<top>,<dk>,<dj>`)

**Correction of section 26's cost**: it said the experiment "makes about 0.8 M lookups, 5-10 ms on the host, 0.6-1.2 s on the LX7". It was written without a timer. Measured, the list decision alone (`best`, mean over the Rogers list of 3 and the report list of 126) is **0.26 ms on the host** with every place and every hypothesis: 20-40 times less than that estimate.

The faster form ranks the places on the 21 Costas symbols (they do not depend on the hypothesis), builds the table of energy shares once for each of the best `top` places, and scores the hypotheses from the table (79 lookups and additions a hypothesis). Detection on the same slots (20 seeds, thresholds from 300 noise slots each: the false-accept counts of 1-3 in 300 are the coarse calibration, not a result):

| form | places | host ms a call | Rogers hits at -24 / -26 dB | report hits at -24 / -26 dB |
|---|---|---|---|---|
| every place, every hypothesis | 77 | 0.26 | 20 / 8 (S1) | 17 / 4 |
| Costas rank, top 3 | 77 | 0.03 | 19 / 7 | 16 / 2 |
| Costas rank, top 3 | 15 (+-1 bin, +-2 rows) | 0.01 | 20 / 13 | 18 / 3 |

* **The rank on the Costas symbols loses nothing measurable** (within the 20-seed noise; the 15-place row looks better because its smaller place set has a lower noise threshold, **and it assumes the place is known to +-1 bin and +-2 rows**, which the test gives it exactly).
* **On the board**: with the window jobs' ratio (about 122, section 21) the full form is about 30 ms, the Costas-ranked forms **about 1-4 ms**; this counts the `ln` of the second statistic, which the LX7 does slowly (S1, the plain share, needs none and hits as often): an estimate, **not measured on the board**.
* Still open as in section 26: the false-accept budget, the place the app really knows, the busy band and fading.

