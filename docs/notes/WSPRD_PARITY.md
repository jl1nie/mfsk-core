# `wsprd` v3.3.0-beta6 parity (Phase 1 of a `wsprd`-compatible daemon)

Goal: through the public API (`Decoder<Wspr>` + `WsprExtras`, `wspr::wsprd_fmt`) reproduce column for column what upstream
`wsprd` v3.3.0-beta6 prints for its options. Oracle: the binary of `scripts/wsprd/build_wsprd.sh`; fixtures
`mfsk-core/tests/fixtures/wsprd_beta6/<case>/` from `scripts/wsprd/gen_fixtures.sh`, golden
`embedded-poc/assets/golden/wspr/150426_0918.wav`. Test: `mfsk-core/tests/wsprd_beta6_parity.rs` (tier B; stdout,
`ALL_WSPR.TXT`, `wspr_spots.txt`, `hashtable.txt` byte-equal, which is also the precision check).

## What matched first try, and what did not

On the first run of the harness, with only the new result fields and the formatter written, 11 of the 13 tests passed:
snr, dt, frequency, drift (truncated, `(int)drift1`), sync, pass, blocksize, jitter, decode type, hard errors, Fano cycles and metric
were already the crate's for `bare`, `f`, `-C 100`, `-C 500 -o 4`, `-qB`, `-s`, `-w`, `-o 1`, `-e 60`, `-H` and the
hashtable. Two cases failed by one hard error / a few hundred Fano cycles on a single decode (G8VDQ under `-z 0.40`; W3BI under
`-C 500 -o 4 -d`), with every soft symbol equal but one: symbol 74, 128 in the crate and 127 in `wsprd`, a value at the
quantisation edge. Dumping the baseband after each subtraction on both sides put the difference in the subtraction of the signals
with a non-trivial `f1`: the crate passed `1500 + f1` through a `float` (1.2e-4 Hz step, up to 0.04 rad over the frame), `wsprd`
keeps `f1`. Subtracted component error 0.3 % on W5BIT (`f1 = -39.6008`). Passing `f1` itself (`subtract_signal_baseband_at`)
made both cases exact. No other numeric difference was found: baseband data of pass 0 differ by 1e-5 (FFT library).

Then the knobs were made real (the crate had none of the six divergences fixed), and the OSD was ported rather than documented:
`-o 1` and `-o 4` are identical on the twelve fresh-table fixtures, but only because OSD accepts nothing there. wsprd's OSD takes a
codeword only if its call is already in `hashtable.txt`, and `wsprd` loads that file. With the table of case `f` preloaded
(`pre*` fixtures), `-o 0`/`-o 1` differ from `-o 2..5` (G8VDQ: 39 vs 33 hard errors, jitter 24 vs 0), all differ from no OSD, and the
old order-2 sweep matches none of them. So `osdwspr.f90` and `indexx.f90` (whose tie order picks the basis) are ported
(`wspr/osdwspr.rs`) and all of `-o 0..5` plus `-C 500 -o 4 -d` and `-o 4 -s` are byte-equal on the preloaded table.

## Divergences fixed (each verified in the C before change)

| # | `wsprd.c` | what |
|---|---|---|
| 1 | `-s` (npasses 1, no subtraction) | did not exist |
| 2 | `ipass==1 && ndecodes_pass==0 && npasses>2` | `-B` stopped after an empty pass 0; the C keeps pass 1 (unobservable on any recording: pass 1 on unchanged data finds nothing) |
| 3 | `ndepth >= 0` | OSD always on, final pass only, its own algorithm; now `-o`, every rung of every pass, `osdwspr` |
| 4 | `fmin`/`fmax` -110..110, `-w` ±150, `fmin += dialfreq_error` | no window; the C adds `-e` **inside the pass loop**, so it accumulates (window `-110+e`, `-110+2e`, `-110+3e`): ported and marked |
| 5 | `-z` | constant |
| 6 | `if (quickmode) break` | already matched |

Not in the list, found on the way and fixed: (a) the subtraction centre rounding above; (b) a decode `unpk_` refuses (`noprint`)
still ends the ladder, counts in `ndecodes_pass` and is not subtracted (the crate went on to the next DT position); (c) the callsign
table is wsprd's 32768 buckets (the crate kept every distinct call), Type 2 stores the call and leaves the locator, the table is
what `hashtable.txt` holds; (d) a Fano decode's `nhardmin` is counted only while subtracting (`-s` leaves 0); (e) the candidate
selection under a window is wsprd's (first 200 maxima in frequency order, then `-d`'s every third bin), not the strongest N.

## Mutations: each case seen to fail

Run with the final tree, one mutation at a time, `cargo test --release --test wsprd_beta6_parity`; "red" are the failing cases.

| knob | mutation | red |
|---|---|---|
| `-s` subtraction | ignore `subtraction = false` | `f_s`, `pre_o4_s` |
| `-s` / `-B` passes | ignore `passes` | `f_B`, `pre_o4_s` |
| `-q` | ignore `jitter` | `f_q` |
| `-C` | ignore `max_cycles_per_bit` | `f_C100`, `f_C500_o4`, and every case whose depth default differs |
| `-d` | ignore `more_candidates` | every case without `-d` (the depth default is on) |
| `-z` | ignore `fano_bias` | `f_z040` |
| `-o n` depth | always 0 | `pre_o2`..`pre_o5`, `pre_C500_o4_d` |
| `-o` every pass | OSD final pass only | `pre_o0`..`pre_o5`, `pre_C500_o4_d` |
| `-o -1` is none | run OSD at depth 0 | `pre` |
| window | ignore `freq_span_hz` | `f_e60` |
| default span | ±50 instead of ±110 | every case on the default span except `f_w` (ND6P at -98 Hz is lost); run before `f_B`, `f_q`, `f_e45` existed |
| `-e` accumulation | shift the window once | `f_e45` |
| subtraction centre | `1500 + f1` through `float` | `f_z040`, `f_C500_o4_d`, `pre_C500_o4_d` |
| Fano `nhardmin` | count without subtraction | `f_s`, `pre_o4_s` |
| formatter | drift rounded; sync rounded; `cycles / 80`; index width 4 | all rows / `hashtable_round_trip` |

Not distinguishable on this recording (mutation stays green), stated rather than hidden: `-w` (every peak the golden has lies
inside ±110 Hz), the `-B` pass-1 rule, and the `noprint` ladder stop (no decode the golden makes is refused by `unpk_`). They are
ported from the C text and covered by nothing but reading.

## Known remaining differences (not exercised by the fixtures)

- Duplicates: `wsprd` drops a decode with the same call within 4 Hz of an earlier one; the crate keeps its own rule (same message,
  5 Hz, one symbol) and has no `-v`, nor the 50-decode cap.
- `wsprd.c:1259`: a candidate whose refined sync stays at or below `minsync1` keeps its *coarse* frequency, shift and sync; the
  crate drops it.
- `fano()` reports a timeout when the search converges on its very last cycle; the crate calls that converged.
- OSD acceptance: the C's probe (`unpackcall` / `unpackgrid` / `unpackpfx` failing breaks the DT loop; `ntype` outside 0..62 is skipped) is
  ported as `msg::wspr::osd_probe`; a Type 2 with an implausible power is accepted by the gate and then `NoPrint`, as in the C.
- The default `Depth` mapping still runs the crate's own OSD (final pass, order-2 sweep): left as it was on request.
