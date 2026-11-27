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
