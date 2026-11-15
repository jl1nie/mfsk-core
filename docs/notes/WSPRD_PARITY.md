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
| 2 | `ipass==1 && ndecodes_pass==0 && npasses>2` | `-B` stopped after an empty pass 0; the C keeps pass 1. Unobservable with a fixed window (pass 1 on unchanged data finds nothing), observable with `-e`, which moves the window between the passes: fixture `far_B_e10` (a call at f1 -126 Hz, outside the window on pass 0, inside on pass 1) |
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
| default span | ±50 instead of ±110 | every case on the default span except `f_w` (ND6P at -98 Hz is lost), including `f_B`, `f_q`, `f_e45`, `far_*` (re-run with them present) |
| `-e` accumulation | shift the window once | `f_e45`, `far_B_e10`, `far_e10` |
| `-B` keeps pass 1 | skip it after an empty pass 0 for `-B` too | `far_B_e10` |
| `-e` implies the span | no implied window | `dial_error_implies_the_span` |
| subtraction centre | `1500 + f1` through `float` | `f_z040`, `f_C500_o4_d`, `pre_C500_o4_d` |
| Fano `nhardmin` | count without subtraction | `f_s`, `pre_o4_s` |
| Type 3 replica | subtract the decoded bits | `t3`, `depth_path_subtracts_the_decoded_codeword`, `the_replica_is_rebuilt_from_the_printed_text` |
| `nhardmin` vs the replica | not counted | `t3` |
| 13-byte hashed-call limit | 12 instead of 10 | `t3_long` |
| `wsprd` duplicate rule | never fires | `t3`, `depth_path_subtracts_the_decoded_codeword` |
| OSD `Break` | `continue` instead of leaving the DT loop | `ladder_tests::an_unpackable_osd_codeword_ends_the_dt_loop` |
| table eviction | buckets that do not collide | `a_colliding_call_evicts_the_earlier_one`, `type2_keeps_the_buckets_locator` |
| Type 2 locator | Type 2 clears the locator | `type2_keeps_the_buckets_locator` |
| formatter | drift rounded; sync rounded; `cycles / 80`; index width 4 | all rows / `hashtable_round_trip` |

Not distinguishable on any recording used (mutation stays green), stated rather than hidden: `-w` (every peak the golden has lies
inside ±110 Hz), and the quantised RMS gate (`wsprd_gates`: the float and quantised RMS differ only within a quantisation step of
the threshold). The `noprint` ladder stop and the replica-refusal `break` are covered by reading and by the unit test of
`replica_info_from_text`, not by a fixture. Mutations were run with `--no-fail-fast` (a failing unit test otherwise hides the
integration binary).

## `wsprd` mode and the `Depth` path

`WsprExtras::wsprd()` sets `replica_from_text` and `wsprd_gates` besides the options; the `Depth` path keeps its own behaviour in
those three places and marks each at the code (CLAUDE.md §2.1):

- **Subtraction replica** (`wsprd.c:1497-1503`): `wsprd` rebuilds the signal from the printed text, so a Type 3 whose call is not in
  the table is subtracted as the codeword of `nhash("...")`, counts `nhardmin` against it, and a text the rebuild refuses ends the
  pass's candidate loop; a hashed call of 11 or 12 characters does not fit `unpk_`'s 13 bytes and is not printed. Measured on a
  synthetic recording (`<K1ABC> FN42AB 37` at -5 dB, `W1AW FN31 30` at -27 dB 3 Hz above): `wsprd` prints only the Type 3, `nhardmin` 48,
  and never hears W1AW; the `Depth` path subtracts the right codeword and hears both (`depth_path_subtracts_the_decoded_codeword`).
  Fixtures `t3` and `t3_long` (an 11-character call in the same bucket: nothing printed). The recording is made by
  `tests/common/wspr_t3.rs` (integer xorshift noise, the crate's synthesiser), written by `examples/gen_wsprd_t3_wav.rs` for
  `gen_fixtures.sh` and rebuilt in memory by the test: only `wsprd`'s outputs are vendored, no WAV. The one libm call is the
  envelope ramp's `cos` (`engine/dsp/envelope.rs:98`); the test pins a hash of the samples, so a synthesiser change asks for a
  fixture rerun instead of looking like a decoder regression.
- **RMS gate** (`wsprd.c:1400-1408`) on the quantised symbols (the Depth path reads the floats), and the **duplicate rule**
  (`wsprd.c:1506-1516`: same call text within 4 Hz, no time test, at most 50), under `wsprd_gates`.
- **OSD**: the crate's own order-2 sweep (final pass) versus `osdwspr` (`WsprOsd::Wsprd`).

## Known remaining differences (each marked at its code)

- `wsprd.c:1325-1343`: a candidate whose refined sync stays at or below `minsync1` keeps its *coarse* frequency, shift and sync in
  `wsprd`; the crate drops it, on every path (`refine_cascade`).
- `fano()` reports a timeout when the search converges on its very last cycle; the crate calls that converged.
- OSD acceptance: the C's probe (`unpackcall` / `unpackgrid` / `unpackpfx` failing breaks the DT loop; `ntype` outside 0..62 is
  skipped) is `msg::wspr::osd_probe`; a Type 2 with an implausible power is accepted by the gate and then `NoPrint`, as in the C.
  The Break path is tested by `ladder_tests::an_unpackable_osd_codeword_ends_the_dt_loop`.
- The replica (`replica_from_text`) keeps the decoded bits for Types 1 and 2 and for the grid half of a Type 3, assuming the
  printed text re-packs to them. It does not always: `unpackcall` turns blanks into NULs (`wsprd_utils.c:191-195`), so a call
  field with an inner blank (`AB1C D` prints `AB1C`; a Type 3 grid slot `N42A F` prints `FN42A`) re-packs to a different
  codeword in `get_wspr_channel_symbols` (`wsprsim_utils.c:165-262`), as does a Type 2 prefix with an inner blank or a short base
  call misread by `pack_prefix` (`:86-111`). Only false decodes have such fields; on one, `wsprd`'s residual, `nhardmin` and
  `hashtable.txt` (`unpk_` of the replica's check data, `wsprsim_utils.c:~292`, not ported) can differ from the crate's.
  Follow-up: port `get_wspr_channel_symbols` as a function of the text.
- The replica-refusal `break` (`decode.rs`, `None => break`) is covered by the unit test of `replica_info_from_text` only;
  no fixture reaches it (documented debt).
- The `wspr-pass2-topn` embedded final pass fills `sync` (its refined sync) and `pass` (2) but none of the other new fields beyond
  what its ladder gives, and it neither subtracts nor deduplicates in `wsprd`'s way.

## Follow-ups

- `embedded-poc/embedded-shared/src/apps/wspr_scan.rs` still subtracts through `subtract_signal_baseband` with `1500 + f1` rounded
  to `f32` (marked there). Moving it to `subtract_signal_baseband_at` changes the CoreS3's runtime path, so it needs a flash and a
  captured log (`embedded-poc/CLAUDE.md`); not done here.
- Tier C (`scripts/run-sensitivity-sweeps.sh wspr`) before the next release: the default-path changes listed in the CHANGELOG
  (subtraction centre, `noprint`, table buckets, `Depth::Fast` pass 1) touch code every WSPR sweep exercises; the corpus is not on
  the machine this was developed on.
