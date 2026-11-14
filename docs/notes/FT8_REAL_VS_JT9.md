# FT8 on real recordings, against `jt9`: what the synthetic corpora did not show

2026-11-03. Trigger: the crates.io crate `rust-ft8` (0.2.1) publishes recall on ft8_lib's 22 recordings. We ran it, `mfsk-core` and
`jt9 -8 -d 3 -L 200 -H 3000` (WSJT-X v3.2.0-rc1) on the same files. Tools and how to rerun: `bench/ft8-compare/README.md`.
The evaluation sheet's "the same sensitivity as `jt9`" rests on one injected signal per file; this is the crowded case.

## Result (22 recordings: ft8_lib `test/wav`, 13 WebSDR + 9 baseline; `jt9` makes 401 decodes)

| decoder | of `jt9`'s 401 | outputs `jt9` lacks |
|---|---:|---:|
| `mfsk-core` before | 383 (95.5 %) | 13 |
| `mfsk-core` with `LlrTail` (this change) | 386 (96.3 %) | 11 |
| + DT at `xdt` (cause 2) | 390 (97.3 %) | 8 |
| + causes 3-6 (#721) | 400 (99.8 %) | 8 |
| + cause 7 (candidate dedupe on the integer-lag DT) | **401 (100 %)** | 7 |
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

## Cause 2, fixed: DT was one 200 Hz sample late (`XDT_OFFSET_S`)

`ft8b.f90`, `ft8_a7.f90` and `ft8c.f90` return `xdt = (ibest-1)*dt2`; `fine_refine_3stage` returned `ibest/200 - 0.5` (its own doc said
`(ibest-1)`). Reported DT and the subtraction start were 5 ms late against `jt9`'s; the symbol extraction (which uses `ibest`, as `jt9`
does) was not. A 5 ms error is a phase error of up to 1.4 rad between the lowest and highest tone of a symbol: a clean +38 dB signal was
subtracted to -15.6 dB here and -55.8 dB by `jt9`; now -55.9 dB. Default strategy: 386 -> 390 of `jt9`'s 401 on the 22 recordings,
outputs `jt9` lacks 11 -> 8, reported DT bias +5.1 -> +0.3 ms, SNR unchanged; 902 crowded synthetic signals 855 -> 861 (`jt9` 860).

**What it uncovered: `sic_rounds` and `decode_block` are the simplified strategies.** Neither re-subtracts with a refined DT between
rounds; `jt9` does (`lrefinedt` when it rebuilds the audio for its second stage), and so does `sic_early`. With the 5 ms error,
a strong signal left 15 dB of residue and a weak signal 10 Hz away stayed decodable by accident (`ZL7CE JR3AX -10` at +6.8 dB beside
`OH9I BG5TKD 73` at +29.5 dB: coarse score 31 in round 1; `jt9`'s own early passes give 3.5 and the new rule 3.5, then 28 after the refined
re-subtraction). With the right alignment the flat strategies trail: `sic_rounds(3)` 902 synthetic 838 -> 828 (`sic_early` 861), `qso3_busy.wav`
20 -> 19 (`WA2FZW DL5AXX RR73`; `sic_early`: all 21 of `jt9`'s), 22 recordings 376 -> 381 (`sic_early` 390). Making `sic_rounds` subtract
with the DT-refining variant brings it to `jt9`'s level (860 / 902, 21 on `qso3_busy.wav`, 380) at 343 ms per file against 194 (`sic_early`: 433);
not done: `sic_rounds` stays the cheap, simplified strategy, and `decode_block` has no budget for a second subtraction. Their pins moved:
`sic_rounds(3)` golden 20 -> 19 messages, the `decode_block` full-parity floor 20 -> 19 (both back to 20 with cause 3 in the host path).

## Causes 3-6, fixed (#721): what the 11 remaining `jt9`-only decodes were

Found stage by stage with the `JT9_TRACE` build and `bench/ft8-compare/ft8-trace-probe.patch`, each checked on the
labelled busy and dense corpora (`scripts/gen_ft8_{busy,dense}_wavs.py`, v3.2.0-rc1 `ft8sim`; 7000 signals, 200
noise-only files): recall equal or higher in every set, no false decode added, the one noise-only decode (also
`jt9`'s) unchanged.

| # | cause | upstream | recovered | effect on 22 files |
|---|---|---|---|---|
| 3 | a re-decoded message was dropped as a duplicate **unsubtracted**: the same signal re-found in a later pass (its second subtraction is where `jt9`'s depth comes from), or a splattering signal's copies (`K4VBM HA8EK RR73` decoded at six frequencies, 2079-2480 Hz, one of them 10 Hz from `K3ZK IK2ZDT RR73` at +2 dB) | `ft8b.f90:463-467` subtracts every valid decode; `ft8_decode.f90` drops the duplicate afterwards | K3ZK, DL8FBD, K6DRY, both OE4ATS, UB3AQS | 390 -> 396, ~+6 % time |
| 4 | the coarse-sync lag grid was two steps (80 ms) off on identical `sync2d` values (`jstrt` rounded 12.5 -> 13 on a 0-based row; window +-63) | `sync8.f90`: `jstrt=0.5/tstep` truncates to 12, 1-based `m`; `JZ=62` | PY1NMG, IZ8IQO (jt9 lag 29, here 27), IT9EJP; UB3AQS lost again (below) | +2, ~+3 % |
| 5 | the blind-CQ AP pass needed `nsync >= 12`, a cost gate of this crate's | iaptype 1 on every candidate past `nsync > syncmin` | CQ DO6AZ (nsync 9, OSD 26 hard errors) | +1, ~+10 % |
| 6 | `<...> DA0FONTANE` decoded by BP and refused: not the empty hash table (the issue's guess), but the callsign grammar's 7-character limit on the `i3=4` call | `ft8b.f90` keeps what `unpack77` renders; `c58` holds 11 characters | DA0FONTANE | +1; random-payload survival +1.3 % (`i3=4` 1.2 -> 4.5 %) |

## Cause 7, fixed: the candidate dedupe compared an interpolated DT

The last one, `CQ UB3AQS KO85` (`191111_110615`, -12 dB, 1047 Hz), sits 9 Hz from one of seven copies of `PA3EPP SP8NFO KN09`
(coarse sync 3330 at 906 Hz, +22 dB). `jt9` decodes that signal three times in its first pass, from the candidates at 906.25,
903.125 and 909.375 Hz (the last two decode the residue the first subtraction leaves, at 905.625 and 906.875 Hz; BP, nsync 15) and
subtracts each: that is the extra depth, and it is what clears UB3AQS (`sync2d` peak 11.3 after the first pass against 7.5 here).
Our coarse sync had both weak candidates (scores 2.19 and 2.06) before the dedupe and dropped them. `sync8.f90:143` removes a candidate
within 4 Hz and `tdiff < 0.04` s of a stronger one, with `candidate0(2,.)=(jpeak-0.5)*tstep`: DT 0.94 and 0.86 against 0.90,
differences of exactly 0.0400 (float32 0.04000002), which survive. This crate added a parabolic sub-lag interpolation to the candidate
DT and deduped on that (0.931, 0.878: differences 0.031 and 0.022), inside the window. Now the dedupe compares the integer-lag DT; the
interpolated one is still what the candidate carries on. The earlier reading here (a subtraction-order effect in a crowded pass) was wrong for this signal.
22 recordings 400 -> 401 of 401; outputs `jt9` lacks 8 -> 7; `sic_rounds(3)` 388 -> 387. Labelled busy (2900 signals) and dense (4102)
corpora: no false decode in any set and none in the 200 noise-only files; recall +3 / -2 signals against the commit before (dense
`real`, 902 signals, identical; `gap` -2, `base` +2, `snr` +1, `busy40` +2). A snapshot row moves its freq/DT: `K1BZM DK8NE -10`
decoded from another candidate, 245.375 -> 244.250 Hz, 22 -> 18 hard errors (`jt9`: 18). The rule applies to candidate lists of 60 or more (`FAITHFUL_DEDUPE_MIN_CAND`; host configs ask for 60-200+, the embedded driver for 30). With the embedded cap the extra near-duplicates pushed `OH3NIV ZS6S -03` (`qso1.wav`, coarse score 2.06, rank 27 of 30 before) out of the list, `ft8_decode_block_real_qso` 4 -> 3 of 5; the lean list keeps the interpolated-DT dedupe, so the embedded driver is unchanged by cause 7. Not compared: `sync8.f90`'s dedupe is pairwise in generation order, this crate's is greedy by score.

Strategies (#722): `sic_rounds(3)` 381 -> 387 of 401 (615 ms per file against `sic_early`'s 1291), `qso3_busy.wav` back to 20 messages
(`sic_early` 21); on the 902 crowded synthetic signals it was 828 -> 837 after causes 3-6 (`sic_early` 862, `jt9` 860) and was not
re-measured after cause 7. `decode_block_multipass` (host, `fft-rustfft`) subtracts a re-found message before dropping it too, and its
full-parity floor on `qso3_busy.wav` is 20 again. The embedded variant of `decode_block` (no `fft-rustfft`) never subtracts at all, so there
is no duplicate subtraction to budget; an earlier version of this note and the floor's comment blamed the embedded budget for it, which
was wrong (the independent review caught it).

Side effect on the cold-start DT bootstrap (`engine::sync::bootstrap_dt_median`, K = 5, +-1.0 s list): its coarse DT moved
+80 ms with cause 4. Against `jt9`'s DT median over the 22 recordings: median error 0.140 -> 0.080 s, files over 130 ms
11 -> 9; on the fixture `191111_110130.wav` 0.120 -> 0.164 s (`ft8_coarse_sync_bootstrap`'s budget 130 -> 180 ms;
fixed-point 0.128 s, under its unchanged 130 ms). Not yet checked on the CoreS3.

Speed, single thread, this box (Ryzen 7 3700X, not the 9900X of `BENCHMARKS.md`): 1.07 -> 1.25 s per file; `jt9` 3.2 s.

## The outputs `jt9` lacks (with the DT change: 8; after #721 8, after cause 7: 7)

No ground truth, so by independent evidence: both callsigns (or the whole text) decoded elsewhere in the corpus by `jt9`; the same
callsigns in an ordered QSO sequence; the reference text; labelled-corpus odds (`bench/ft8-compare/phantom_stats.py` on `ft8_busy_sweep`,
420 files: 1-2 phantoms in ~2380 decodes, all OSD, 27+ hard errors, nsync <= 9; none in 2100+ BP or 30 AP decodes); recovery over 16
copies with 0.5 sigma extra noise (known phantoms 0-2/16; confirmed weak signals mean 0.76). Seven of eight are real by that evidence
(two in the reference text); `SP7NHS YC2FCE -15` is undecided (no callsign seen elsewhere, 6/16). The phantoms seen along the way
(`35J89OLNIMBS5`, `CQ F5ADE JN06`, `0K8SEL/P BB6PSE R CQ78`, `GJ0KYZ VK2LAW QF56`) are unattested and transient: present in one
configuration only. A "low nsync with high SNR" rule does not separate them (44 confirmed decodes have nsync <= 9 and median SNR -1 dB).

After #721 (8, a different set): gone, all at the edge of `jt9`'s own reach, `GM0LIR UA9SIX -09` (DT +2.5, past the
+-62-lag window now that it is `sync8.f90`'s), `R2ATW IZ0VLL -08`, `EA8BEV LU3DW RR73`; new, each attested:
`CU2DX R2DQA KO96` (CU2DX on 1011 Hz, DT 0.2 in other recordings; KO96 is an R2 grid), `EA8PP JH0INP PM96` (`jt9`'s exact
text in another recording, same 940 Hz / DT -0.6), `DK5OK DB4BU R-04` (the QSO `jt9` decodes next: `DB4BU DK5OK RR73`,
`DK5OK DB4BU 73`, 1822 Hz), `EA8PP JA6VQA -16` (OSD at 21 hard errors; `JA6VQA EA8PP R-24` / `73` elsewhere). Kept:
`SP7NHS YC2FCE -15` (still undecided), `CQ EA5OL IM99`, `UA9SIX GM0LIR R-07`, `CQ CU2DX HM77`.

## In tier B since 0.16 (2026-11-14), against v3.3.0-beta6's `jt9`

All 22 recordings are vendored (`embedded-poc/assets/`) and `mfsk-core/tests/ft8_real_recordings_vs_jt9.rs` checks each against
every line beta6's `jt9 -8 -d 3 -L 200 -H 3000` prints (`tests/fixtures/ft8_lib_jt9_beta6.tsv`), so this comparison runs on every PR
instead of by hand at a release. beta6 prints 402: rc1's 401 and `EA8PP JA6VQA -16` in `websdr_test5`, one of the outputs listed
below as attested, which `jt9` now makes too. The crate finds 402 of 402; its outputs `jt9` lacks are the six still listed below,
named in the test (any other fails as a phantom).

SNR on the 402: mean -0.10 dB, median -0.10, 90 % within -2.0..+1.6 dB; 20 rows past 3 dB and 8 past the test's 4 dB, held out of
the SNR check as debt. They cluster by recording: every row of `websdr_test4` reads 3-4 dB low, of `websdr_test7` 3-5 dB high, and the
two hashed `<...> RY8CAA` rows 5-6 dB high. On the synthetic dense corpus above, the two agree within 1 dB in 3 640 of 3 644, so the
difference is something real audio has and white noise does not (a per-file baseline is the candidate; not measured; #747).

## Limits

One band session, 22 files, one reference decoder; the labelled corpora are white noise. The `jt9` trace build and our local trace patch
are write-only (22 files decode identically with them). Tier C was not run.

## SNR against `jt9` (dense corpus, 200 files, before cause 7)

Reported SNR of the 3 644 decodes both make, `jt9` v3.2.0-rc1 and `Decoder` default: mean difference ours - `jt9` +0.02 dB (sd 0.31, mean absolute 0.10), identical in 3 297, within 1 dB in 3 640. Both sit about 0.4 dB under the injected SNR (ours -0.39, `jt9` -0.42, sd 0.57 / 0.59); by band the bias runs from +0.1 dB (-30..-20) to -0.6 dB (above 0). Ours decoded 25 signals `jt9` did not, `jt9` 3 that ours did not.

## Open work

[#722](https://github.com/jl1nie/mfsk-core/issues/722): what `sic_early`, `sic_rounds` and `decode_block` should each promise (the pins, the crowded-neighbour regression test, `Depth::Fast` and `decode_block` docs, the CoreS3 check of the -5 ms DT). [#721](https://github.com/jl1nie/mfsk-core/issues/721) (the causes and the method) is addressed by PR #723; its remaining to-do is the real-recording row in `BENCHMARKS.md`.
