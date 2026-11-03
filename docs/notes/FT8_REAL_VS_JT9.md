# FT8 on real recordings, against `jt9`: what the synthetic corpora did not show

2026-11-03. Trigger: the crates.io crate `rust-ft8` (0.2.1) publishes recall on ft8_lib's 22 recordings. We ran it, `mfsk-core` and
`jt9 -8 -d 3 -L 200 -H 3000` (WSJT-X v3.2.0-rc1) on the same files. Tools and how to rerun: `bench/ft8-compare/README.md`.
The evaluation sheet's "the same sensitivity as `jt9`" rests on one injected signal per file; this is the crowded case.

## Result (22 recordings: ft8_lib `test/wav`, 13 WebSDR + 9 baseline; `jt9` makes 401 decodes)

| decoder | of `jt9`'s 401 | outputs `jt9` lacks |
|---|---:|---:|
| `mfsk-core` before | 383 (95.5 %) | 13 |
| `mfsk-core` with `LlrTail` (this change) | 386 (96.3 %) | 11 |
| `rust-ft8` 0.2.1, passes 2 / passes 3 + deep | 303 (75.6 %) / 326 (81.3 %) | 11 / 19 |

`rust-ft8`'s own 88.95 % is against ft8_lib's text files; on those `jt9` itself gets 71 % (they hold signals `jt9` never makes), so the text
files are not a reference. Speed (single thread, per file): `jt9` ~1.0 s, `mfsk-core` ~0.43 s, `rust-ft8` 0.06-0.17 s.

The 18 decodes `mfsk-core` missed concentrated in the rungs `jt9` needs: it made 275 of its decodes by BP (2 lost), 115 by OSD (12 lost),
11 by AP1 (4 lost). The synthetic corpora hold almost only BP decodes, hence no difference there (`scripts/gen_ft8_dense_wavs.py`:
SNR, spacing and DT drawn from these recordings, one factor at a time; all five sets within noise of `jt9`).

## Cause 1, fixed: the last group of each half (`LlrTail`)

`ft8b.f90` `do k=1,29,nsym` starts the last `nsym` > 1 group at the first uncovered symbol and runs it on into the next (Costas) symbol.
`engine::llr::fill_bmet_for_nsym` re-anchored it on the last data symbols (made for FT4, #18). On identical audio the all-sign agreement of
`llrb` / `llrc` / `llre` with `jt9`'s traced LLRs was 45 / 40 / 66 % (bits 79-87, 166-174); now 94 / 94 / 95 %. A decode at 29 hard errors
(`DO2JHE LZ1JZ -13`) is decided by it: `jt9`'s LLRs through this crate's OSD decode it, this crate's own did not.
The complex symbol spectra `cs` are identical on both sides (checked), so the downsample path is not the cause.

## Found, not landed: DT is one 200 Hz sample late

`ft8b.f90` returns `xdt = (ibest-1)*dt2`; `fine_refine_3stage` returns `ibest/200 - 0.5` (its own doc says `(ibest-1)`). Reported DT and the
subtraction start are 5 ms late against `jt9`'s, the symbol extraction (which uses `ibest`) is not. A clean +38 dB signal is subtracted to
-15.6 dB here and -55.8 dB by `jt9`; with the one-sample change -55.9 dB. Experiment (not in the tree): 390 / 401, outputs `jt9` lacks 8,
reported DT bias +5.1 -> +0.3 ms, SNR unchanged. Needs the snapshot DT columns, the consumers of `DecodeResult::dt_sec` as a start time,
`decode_block`, and a review.

## Still different (after `LlrTail`: 15 `jt9`-only decodes)

policy rejects a decoded `<...> DA0FONTANE` (empty hash table): 1; no candidate: 1; sync gate: 2-3; BP/OSD finds no codeword on the same
candidate, 7-9 (LLRs equal in 2, differing even in `llra`/`llrd` in 8: the residual after subtraction). Candidate lists agree 88.8 % on the
first search (same scores, so the coarse sync is the same measure) and 61-69 % after subtractions.

## The outputs `jt9` lacks (with the DT change: 8)

No ground truth, so by independent evidence: both callsigns (or the whole text) decoded elsewhere in the corpus by `jt9`; the same
callsigns in an ordered QSO sequence; the reference text; labelled-corpus odds (`bench/ft8-compare/phantom_stats.py` on `ft8_busy_sweep`,
420 files: 1-2 phantoms in ~2380 decodes, all OSD, 27+ hard errors, nsync <= 9; none in 2100+ BP or 30 AP decodes); recovery over 16
copies with 0.5 sigma extra noise (known phantoms 0-2/16; confirmed weak signals mean 0.76). Seven of eight are real by that evidence
(two in the reference text); `SP7NHS YC2FCE -15` is undecided (no callsign seen elsewhere, 6/16). The phantoms seen along the way
(`35J89OLNIMBS5`, `CQ F5ADE JN06`, `0K8SEL/P BB6PSE R CQ78`, `GJ0KYZ VK2LAW QF56`) are unattested and transient: present in one
configuration only. A "low nsync with high SNR" rule does not separate them (44 confirmed decodes have nsync <= 9 and median SNR -1 dB).

## Limits

One band session, 22 files, one reference decoder; the labelled corpora are white noise. The `jt9` trace build and our local trace patch
are write-only (22 files decode identically with them). Tier C was not run.
