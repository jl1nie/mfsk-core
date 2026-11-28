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

* **The 15 s slot is not the constraint**: faithful Deep is 0.4 s a slot (worst 0.5 s) on this host; at 4-6 times the instruction time that is a few seconds on a Pi 3B. **Threads do not help** as assumed: 4 threads gave 1.07x (faithful) and 1.0x (cells) here, against the 60-80 % efficiency `PROFILES.md` assumed.
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

**What it means for a Pi 3B class**: the cost is not the constraint (the faithful pipeline is 0.4 s a slot on this host). The measured design that reaches `jt9`: the cells flow as a **fast candidate generator** (its search is not what fails: 50 of 60 misses have a candidate in the list) and the **faithful per-candidate decode and subtraction** for the candidates, which is the hybrid of section 1 (99.2 % of the 1 338, 475 ms a slot on this host).
