# ft8-compare

Not part of the workspace (root `Cargo.toml` `exclude`). Runs `mfsk-core` and the crates.io crate `rust-ft8` (pinned `=0.2.1`) on the same
WAV files and compares both with WSJT-X's own `jt9`, stage by stage. What it was made for and what it found:
[`docs/notes/FT8_REAL_VS_JT9.md`](../../docs/notes/FT8_REAL_VS_JT9.md). Open work: the issues linked from that note.

## The data

| set | what | reference | where |
|---|---|---|---|
| **22 real recordings** | ft8_lib `test/wav`: `websdr_test1..13.wav` and `191111_*.wav` (the 22 that have a `.txt`), 12 kHz mono 16-bit, 15 s, MIT | `jt9 -8 -d 3 -L 200 -H 3000` (WSJT-X **v3.2.0-rc1**) on the same files: 401 decodes. Not ft8_lib's `.txt` (on those `jt9` itself gets 71 %) | `git clone https://github.com/kgoba/ft8_lib` |
| `qso3_busy.wav` and the 22 ft8_lib recordings | in this repo, `embedded-poc/assets/` (since 0.16) | `jt9`; the 22 are tier B (`mfsk-core/tests/ft8_real_recordings_vs_jt9.rs` against beta6's `jt9`, `tests/fixtures/ft8_lib_jt9_beta6.tsv`), so recall and phantoms on them are checked on every PR; this bench is for the stage-by-stage comparison and rust-ft8 | repo |
| **busy-band corpus** `ft8_busy_sweep` | 420 files, 10/20/40 signals, 100 single, 200 noise-only; `truth.csv` | the messages sent | `scripts/gen_ft8_busy_wavs.py` (needs `ft8sim` of v3.2.0-rc1) |
| **dense corpus** `ft8_dense_sweep` | 200 files, five sets (`base`, `snr`, `gap`, `dt`, `real`) whose SNR, spacing and DT are drawn from the 22 recordings, one factor per set (paired: same messages, frequencies and noise); `truth.csv` | the messages sent | `scripts/gen_ft8_dense_wavs.py` |

The generated corpora are not in git (`.gitignore`); regenerate with `<ft8sim> <out-dir>`. `ft8sim` must be the v3.2.0-rc1 build
(`scripts/build_ft8sim.sh <v3.2.0-rc1 tree> target/ft8sim-3.2`); the default `target/ft8sim` is the older tree the fading corpora use.

## The comparison

A decode of ours **matches** a reference entry when the message text is equal (whitespace and case ignored) and the frequency is within 10 Hz
(5 Hz against a truth list); each reference entry is used once. DT is not compared against `jt9` to match (rust-ft8 shifts it by its
`window_start_offset`). Reported per decoder: how many of the reference entries it has (**recall**), and how many outputs the reference
does not have (**extras**; against a truth list, those are phantoms). Timing: single thread, one file at a time.

## Rerun

```sh
# 1. the reference decoder, once
scripts/build_jt9_upstream.sh ../WSJT-X v3.2.0-rc1                      # -> target/upstream/build-v3.2.0-rc1/jt9
JT9_TRACE=1 scripts/build_jt9_upstream.sh ../WSJT-X v3.2.0-rc1          # -> ...-trace/jt9: same decodes, plus ft8_trace.dat (never time with it)

# 2. the 22 recordings
WAVS=$(ls ft8_lib/test/wav/*.txt | sed 's/\.txt$/.wav/')
bench/ft8-compare/run_jt9.sh target/upstream/build-v3.2.0-rc1/jt9 run $WAVS          # run/jt9_<stem>.out, run/jt9_wall.txt
(cd bench/ft8-compare && cargo build --release)
RAYON_NUM_THREADS=1 bench/ft8-compare/target/release/ft8-compare run/out.csv $WAVS  # mfsk, rf8-default, rf8-deep
python3 bench/ft8-compare/compare.py run/out.csv run ft8_lib/test/wav               # recall / extras against jt9 and the .txt; run/diff.txt lists every difference
#   FT8_COMPARE_STRATEGIES=1 adds mfsk-sic3 and mfsk-single; FT8_COMPARE_ONLY=mfsk|sic3 runs one of ours.

# 3. against a truth list (dense / busy corpus)
python3 scripts/gen_ft8_dense_wavs.py target/ft8sim-3.2/ft8sim embedded-poc/assets/ft8_dense_sweep
bench/ft8-compare/run_jt9.sh target/upstream/build-v3.2.0-rc1/jt9 dense embedded-poc/assets/ft8_dense_sweep/*.wav
RAYON_NUM_THREADS=1 bench/ft8-compare/target/release/ft8-compare dense/out.csv embedded-poc/assets/ft8_dense_sweep/*.wav
python3 bench/ft8-compare/score_truth.py embedded-poc/assets/ft8_dense_sweep/truth.csv dense/out.csv dense ft8_dense_
#   phantom_stats.py: false decodes by method / hard errors / nsync on ft8_busy_sweep (needs our traces, below).
```

## Stage by stage (why a decode is missing or extra)

`jt9` side: the `JT9_TRACE` build writes, per file, `ft8_trace.dat` (`STAGE`/`PASS`/`CAND`/`GATE`/`FAIL`/`DEC`/`REJ`/`SUB`, the five LLR sets of
every candidate that passes the gate, and with `JT9_TRACE_CS_F=<Hz>` the complex symbol spectra, `JT9_TRACE_SYNC_F=<Hz>` the `sync2d` row of
a bin) and the residual after every search pass (`resid_<nzhsym>_<pass>.bin`). `run_jt9.sh` collects them as `trace/<stem>.trace`, `resid/`.

Our side: `ft8-trace-probe.patch` (write-only, env-gated; `git apply` it on the tree you measure; **never commit it**) adds
`MFSK_FT8_TRACE=<file>` (stages A/C, rounds, `CAND`, `GATE`, `NOACC`/`ACC`/`REJ_*`, `DEC`, `SUB`, `DUPE`), `MFSK_FT8_TRACE_LLR=1`,
`MFSK_FT8_TRACE_CS_F`, `MFSK_FT8_TRACE_SYNC_F` (Hz) and `MFSK_FT8_DUMP_RESID=<dir>`. `run_ours.sh <bin> <dir> <wav>...` writes
`<dir>/rtrace/<stem>.{csv,rtrace}`. All the analysis scripts read `<dir>/trace/<stem>.trace` (jt9) and `<dir>/rtrace/<stem>.rtrace` (ours),
`<dir>/jt9_<stem>.out`, `<dir>/files.txt` (the WAV list):

| script | answers |
|---|---|
| `stage_trace.py <dir>` | for every `jt9` decode: which `jt9` stage / pass / LLR set / BP-or-OSD made it, and what we did with the same signal (decoded, no candidate, gate, ladder, dropped); by rung |
| `extras_trace.py <dir>` | for every output `jt9` lacks: how we made it, and what `jt9` did at the same spot |
| `llr_compare.py <dir> <stem> <message>`, `llr_stats.py`, `llr_bitpos.py` | LLR sign agreement with `jt9` for a candidate, over all candidates, by bit position |
| `cs_compare.py`, `cand_compare.py`, `resid_compare.py` | complex spectra, coarse candidate lists, subtraction depth per signal (numpy) |
| `phantom_stats.py`, `score_truth.py`, `solo.py` | false decodes by feature; truth scoring; one signal of a corpus file on its own |
| `src/bin/osd_probe.rs` | feeds `jt9`'s LLRs to this crate's `bp_llr_zsum` + `osd174_91` |

A method that proved itself: for a candidate where both decoders have the same LLRs, the difference is in the BP/OSD/AP step; where the LLRs differ,
compare the spectra (`cs_compare.py`), then the residual (`resid_compare.py`). The scripts that need numpy say so in their header.
