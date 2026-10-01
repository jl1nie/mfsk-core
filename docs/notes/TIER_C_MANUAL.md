# Tier C: the benchmark manual

How this crate's sensitivity and speed are measured, how the corpora behind
the measurements are set up, and what went wrong often enough to be written
down. Results live in `BENCHMARKS.md` and the per-protocol `*_BENCHMARK.md`.
This file is the method. `CLAUDE.md` says *when* tier C runs (before a
release tag, and after a change that plausibly moves sensitivity). This says
*how*.

## 1. Quick reference

```sh
scripts/release-status.sh                 # which protocols need re-sweeping, and why
scripts/run-sensitivity-sweeps.sh         # everything present (about 9 min on a 9900X)
scripts/run-sensitivity-sweeps.sh ft4 q65 # or just what moved
# once the numbers are understood:
python3 scripts/sweep-regression-check.py --update-baseline target/sweep-csv/*.csv
```

On a machine without corpora, set up once (§3): build the simulators, build
the upstream reference, generate the corpora. About 18 GB, a few minutes of
wall clock plus the WSJT-X CMake build.

## 2. Principles

These are the rules the tooling enforces, or that it cannot enforce and
has had to learn. Each one was broken at least once.

1. **Define the task before comparing with upstream.** A task is the job
   (band, what the operator knows, depth), with the upstream command line
   and the crate request that do it, side by side in
   `scripts/upstream_tasks.json`. "Both at their defaults" is not a task.
   Q65's first `jt9` comparisons differed in three ways. `jt9 -3` tries its
   Rx frequency first, and the corpus signal sat exactly there. The crate
   scanned 200–3000 Hz. And the crate ran AP as a second full scan, where
   `jt9` runs AP inside its candidates. None of those comparisons measured
   anything.
2. **Compare on the same files, paired per trial.** Upstream's per-trial
   outcome on each corpus is committed (`docs/notes/upstream/`), and every
   group is tested for "upstream decodes significantly more files than
   the crate" (exact McNemar). A 50 % crossing from 5–20 trials per cell is a
   summary, not a test. Q65 D-120 plain read ±0.00 dB on the 5-trial
   release corpus and moved 0.40 dB at 60 trials.
3. **Two baselines, two questions.** `docs/notes/sweep-baseline.json` asks
   whether the crate got worse than it was. `docs/notes/upstream/*.csv` asks
   whether it is worse than WSJT-X. Only the second could have caught Q65
   running 2.9–4.5× slower than `jt9` since before #552.
4. **Time the same sequence, one file at a time, single-threaded.** For
   `jt9`, use `timer.out`'s total, which leaves out process start, with a
   persistent working directory so FFTW wisdom stays warm. A fresh
   directory per file made `jt9` look 1.5–2.5× slower than it is. Both sides
   must do the same work: no-AP, then AP only if nothing decoded.
5. **An A/B of two builds runs two binaries.** Put both variants in one
   build behind an environment toggle, or give each worktree its own
   `CARGO_TARGET_DIR`. Two worktrees sharing one target dir timed a build
   against itself and reported a 2× cost as "0–2 %", which was pushed.
6. **Check that a run did the work.** Count the rows a sweep wrote. A sweep
   test binary run outside `cargo test` has no `CARGO_MANIFEST_DIR`, finds
   no corpus, and passes having decoded nothing. The sweep CSVs append, so
   delete one before reusing its path.
7. **Measure a cause before naming it.** Profile (`perf`), then count the
   work (candidates, BP runs) before proposing a fix. The Q65 slowdown was
   8 candidates on every scan, noise included, which a count showed in
   one run.
8. **More trials go in a scratch directory, not the corpus.** To resolve a
   cell, generate more trials of it into `target/` and pair them there.
   Deterministic simulators (`q65sim`, `fst4sim`, and every seeded one) draw a
   cell in one sequence, so the first N trials of a longer run equal the
   corpus, and the rest are independent. The committed corpora do not grow.
9. **Say where a number came from**: the upstream tag, flags and depth; the
   corpus stamp; the machine; trials per cell.

## 3. Setup

### 3.1 Which WSJT-X, for what

| use | tree | why |
|---|---|---|
| the `ccir_*` / AWGN corpora (`ft8`, `ft4`, `fst4`, `jt9`, `wspr` simulators) | `2b9d654` | WSJT-X corrected its Watterson fading model in February 2024. A newer tree makes a harsher channel under the same `ccir_*` name (§3.1.1) |
| `jt65sim`, `q65sim` | the WSJT-X CMake build (`$HOME/wsjtx-build` by default) | they link all of `wsjt_fort` / `wsjt_cxx` |
| the ITU corpus (`ft8_itu`), the busy-band corpus (`ft8_busy`), `sjtty` | `v3.2.0-rc1` | ITU channel codes and JTTY exist only in 3.x; the busy-band generator parses 3.x `ft8sim` output |
| the upstream reference `jt9` / `wsprd` | `v3.2.0-rc1` | the decoders this crate ports (#439, #442) |

Take a tree with `git archive`, which leaves the clone alone:

```sh
git -C ../WSJT-X archive 2b9d65408 | tar -x -C target/upstream/wsjtx-2b9d654   # mkdir it first
```

#### 3.1.1 Background: the trees

"Same simulator build" has a concrete answer, and `../WSJT-X` is no longer it.

**The existing corpora need the `2b9d654` tree** (2024-03-12; the tip of the
mirror this repo tracked until 2026-09). Checked on 2026-09-25: an `fst4sim`
built by `scripts/build_fst4sim.sh` from a `2b9d654` checkout regenerates
`fst4_60_ccir_poor_m24_{01,05,20}.wav` byte for byte, and one built from
`b4f9a43` (WSJT-X 2.7's line, what `../WSJT-X` is now) reproduces the AWGN file
but **not the fading files**. WSJT-X corrected the Watterson simulator in
February 2024 (`f21f37ad0` "Correct the definition of fspread", `7ce6e29a7`
"same spreading function as that used in ITU report ITU-R F.1487"). For the
same `fspread` argument a newer tree gives a *wider* Doppler spectrum, so
`ccir_*` regenerated from any tree that has those commits is a different,
harsher channel under the same name, and AWGN files matching byte for byte
hides it. The `ccir_good/moderate/poor` numbers (0.1/0.5, 0.5/1.0, 1.0/2.0 Hz
and ms) are therefore **milder than the ITU channels of the same numbers**.
Nothing here is wrong by that; it means a baseline is only comparable across
corpora made from the same tree.

    git -C <WSJT-X> archive 2b9d65408 lib | tar -x -C <dir>/wsjtx-2b9d654
    scripts/build_fst4sim.sh <dir>/wsjtx-2b9d654 <dir>/fst4sim     # ft4sim, ft8sim, ... likewise

**The ITU fast-fading corpus (`ft8_itu_sweep/`) needs a 3.x tree**, because the
ITU channel codes (`LQ LM LD MQ MM MD HQ HM HD`) are an `ft8sim` 3.x feature,
and it uses the corrected Watterson. Its channels are named `itu_*`, never
`ccir_*`, so the two are not confused, and `sweep-baseline.json` keeps them
under `ft8_itu/...`:

    git -C <WSJT-X> worktree add <dir>/wsjtx-3.2 v3.2.0-rc1
    scripts/build_ft8sim.sh <dir>/wsjtx-3.2 <dir>/ft8sim-3.2
    FT8_CHANNEL_SET=itu scripts/gen_ft8_sweep_wavs.sh <dir>/ft8sim-3.2/ft8sim \
        embedded-poc/assets/ft8_itu_sweep

`build_ft8sim.sh` needed changes for 3.x (two new 77-bit modules, `gfsk_pulse.f90`
moved out of `lib/ft2/`, `db()` and `print_version()`); it handles both. The
3060 files (9 channels x 17 SNR points x 20 trials) are deterministic. To check a
regeneration, on the machine that made the reference (gfortran 11.4, glibc,
`MFSK_SIM_SEED` unset):

    md5sum embedded-poc/assets/ft8_itu_sweep/ft8_itu_mm_m20_01.wav   # f36e24bca649c8db84a48ffbfdd11118
    md5sum embedded-poc/assets/ft8_itu_sweep/ft8_itu_mq_m18_10.wav   # 5a6253d3376a646103f8d28fde5ee25d

What the corpus is for: FT8's crossings on the quiet and moderate channels sit
near -19.4 to -20.8 dB, but the disturbed ones (`itu_ld`, `itu_hm`: 10 Hz of
Doppler spread; `itu_hd`: 30 Hz) are wider than FT8's 6.25 Hz tone spacing and
only cross at -5.0 and -1.7 dB, or never (`itu_hd` is 0 % at +10 dB). That is the
regime the `ccir_*` set never reaches (its widest spread is 1.0 Hz in the old
definition), and the one where the choice of how many symbols to combine
coherently matters most.

### 3.2 Building the simulators

A WSJT-X clone (the scripts default to `../WSJT-X`, beside this repo) and `gfortran`,
`gcc`, `g++`, `cmake`. The two CMake-built simulators additionally need
what a full WSJT-X configure wants — Qt5, hamlib, boost, fftw3 — even
though neither links the GUI.

`scripts/build_*sim.sh` assemble a minimal gfortran subset of WSJT-X's
`lib/` — no CMake, no Qt, seconds each. Build, then generate:

```sh
scripts/build_ft8sim.sh            # then gen_ft8_sweep_wavs.sh
scripts/build_ft4sim.sh            # then gen_ft4_sweep_wavs.sh
scripts/build_fst4sim.sh           # then gen_fst4_sweep_wavs.sh
scripts/build_jt9sim.sh            # then gen_jt9_sweep_wavs.sh
scripts/build_wsprsim.sh           # then gen_wspr_sweep_wavs.sh
scripts/gen_ft8_sweep_wavs.sh      # each takes [sim-path] [out-dir]
```

Each `gen_*` script takes an optional simulator path and out-dir, skips
cells already present, and parallelises over `JOBS` (default `nproc`).
Re-running one reproduces the same corpus; generate into a **separate
out-dir** rather than in place when changing `TRIALS` or `MFSK_SIM_SEED`,
since either redraws a whole cell and reshuffles which signal each trial
index names.

Pass the tree explicitly rather than relying on `../WSJT-X` pointing at the
right commit: `scripts/build_ft8sim.sh <tree> <out-dir>`. Every generator
refuses a simulator not linked with `scripts/sim_sgran_stub.c` (§3.4), so an
unseeded build is caught, but a build from the wrong tree is not.

These two link the whole `wsjt_fort` + `wsjt_cxx` libraries rather than a
pickable subset, so they come from WSJT-X's own CMake build:

```sh
cmake -S /path/to/WSJT-X -B ~/wsjtx-build -DCMAKE_BUILD_TYPE=Release \
      -DWSJT_GENERATE_DOCS=OFF -DWSJT_SKIP_MANPAGES=ON
cmake --build ~/wsjtx-build --target jt65sim q65sim -j"$(nproc)"

scripts/build_jt65sim.sh            # re-link against the seeding stub
scripts/gen_jt65_sweep_wavs.sh target/jt65sim/jt65sim
scripts/gen_q65_sweep_wavs.sh  ~/wsjtx-build/q65sim
```

`q65sim` is used straight from the CMake tree; `jt65sim` is not. CMake's
`jt65sim` takes `sgran_` out of `libwsjt_fort.a` and so re-seeds per run,
which `scripts/build_jt65sim.sh` fixes by re-linking the same objects with
`sim_sgran_stub.o` passed explicitly — the linker resolves the symbol
before it searches the archive. Seconds, no recompilation.

**`-DWSJT_SKIP_MANPAGES=ON` is not optional**: configure hard-errors
without it unless `a2x` (asciidoc's, *not* `asciidoctor`'s) is installed,
and the manpages are of no use here. `-DWSJT_GENERATE_DOCS=OFF` is the
same bargain for the handbook. Only the two Fortran/C++ libraries actually
compile — the Qt GUI is never a dependency of these targets — so the build
is ~2 min, and the same tree also yields the real `jt9` CLI
(`cmake --build ~/wsjtx-build --target jt9`) that several sections below
compare against.

### 3.3 Building the upstream reference

```sh
scripts/build_jt9_upstream.sh            # target/upstream/build-v3.2.0-rc1/{jt9,wsprd}
```

It exports `v3.2.0-rc1` with `git archive`, applies the four CMake edits
this host needs (none touches a decoder), and builds `jt9` and `wsprd`.
Check the result: `jt9 -8 -d1/-d2/-d3` on `embedded-poc/assets/qso3_busy.wav`
decodes 14 / 20 / 21. `docs/notes/upstream/*.csv` records the binary's
sha256.

### 3.4 Generating the corpora

Each generator takes `[simulator] [out-dir]`. Generate into an **empty**
directory. A generator only fills in missing cells, and it refuses to add
to WAVs that carry no stamp or another seed.

| corpus | command | WAVs | size |
|---|---|---|---|
| `ft8_sweep` | `scripts/gen_ft8_sweep_wavs.sh <ft8sim from 2b9d654>` | 1040 | 358 M |
| `ft8_itu_sweep` | `FT8_CHANNEL_SET=itu scripts/gen_ft8_sweep_wavs.sh <ft8sim from 3.2> embedded-poc/assets/ft8_itu_sweep` | 3060 | 1.1 G |
| `ft8_busy_sweep` | `scripts/gen_ft8_busy_wavs.py <ft8sim from 3.2> embedded-poc/assets/ft8_busy_sweep` (numpy) | 420 + `truth.csv` | 145 M |
| `ft4_sweep` | `scripts/gen_ft4_sweep_wavs.sh <ft4sim from 2b9d654>` | 1040 | 147 M |
| `fst4_sweep` | `scripts/gen_fst4_sweep_wavs.sh <fst4sim from 2b9d654>` | 5120 | 13 G |
| `wspr_sweep` | `scripts/gen_wspr_sweep_wavs.sh <wsprsim>` | 260 | 716 M |
| `jt9_sweep` | `scripts/gen_jt9_sweep_wavs.sh <jt9sim>` | 300 | 372 M |
| `jt65_sweep` | `scripts/build_jt65sim.sh; scripts/gen_jt65_sweep_wavs.sh target/jt65sim/jt65sim` | 300 | 372 M |
| `q65_sweep` | `scripts/gen_q65_sweep_wavs.sh $HOME/wsjtx-build/q65sim` | 1320 | 2.0 G |
| `jtty_sweep` | `scripts/gen_jtty_sweep_wavs.sh <WSJT-X clone>` (builds `sjtty` from `v3.2.0-rc1`) | 360 | 34 M |

`jt65_sweep` and `fst4_sweep` have a few upstream bookkeeping files
tracked in git (`avemsg.txt`, `decoded.txt`, …). Move the WAVs, not
the directory, when regenerating in place.

#### 3.4.1 Background: what makes a corpus reproducible

Every AWGN/fading 50%-crossing on this page comes from a `*sim`-generated
corpus under `embedded-poc/assets/*_sweep/` — ~17 GB, gitignored per
directory, and absent on CI, which is why `scripts/run-sensitivity-sweeps.sh`
exists at all. A fresh clone has none of it. This section is how to put it
back on a machine; the per-protocol `FT8_BENCHMARK.md` / `FT4_BENCHMARK.md` /
`FST4_BENCHMARK.md` carry the same steps for their own protocol in more
detail, and JT9/JT65/Q65/WSPR had none until 2026-09-23.

**Two properties of these corpora decide how the rest of this section
reads.** Neither is obvious, and both were measured rather than assumed
(2026-09-23, on the Ryzen 7 3700X box):

- **Upstream re-seeds five of the seven simulators; this repo stops it.**
  `gran_()` draws its noise from C `rand()`, and upstream's `sgran_()`
  seeds that from /dev/urandom, so `ft8sim`, `ft4sim`, `jt9sim`, `jt65sim`
  and `wsprsim` wrote a different realisation on every run. `fst4sim` and
  `q65sim` never did — `fst4sim.f90:109` is `!   call sgran()`, commented
  out upstream, and `q65sim.f90` has no such call. The build scripts here
  link `scripts/sim_sgran_stub.c` in place of `lib/sgran.c`, which seeds
  from `MFSK_SIM_SEED` (default 1), so **all seven now regenerate
  byte-identically**. Verified by running each generator twice and diffing
  the WAVs.
- **A corpus is therefore a function of (simulator binary, arguments,
  seed), and a baseline travels with it.** That is what makes
  `sweep-baseline.json` comparable between machines at all. Before the
  stub it was not: re-measuring on a second box against a baseline taken
  on the first showed the re-seeded protocols scattering by −0.56 to
  +0.68 dB with the decoder proven byte-identical, while all 20 of Q65's
  groups — the one deterministic generator in that run — landed on their
  stored values to the last printed digit, even though the two corpora were
  not the same size (2 640 trials here against the stored 2 820: the cells
  they do share are bit-identical, and the surplus sat on plateau points
  that an interpolated 50%-crossing does not see).
- **Determinism does not shrink the sampling error, it shares it.** At 20
  trials per cell the draw-to-draw spread of a 50%-crossing is larger than
  it looks: five independent FT4 corpora gave sd 0.08-0.27 dB per channel,
  but a sixth draw moved FT4's `ccir_poor` 0.43 dB, and a single redraw
  moved FT8's `ccir_poor` **1.32 dB** — same decoder, same grid. Fading
  channels are the heavy tail. So a fixed seed buys reproducibility, not
  accuracy: these numbers are exact for detecting a code change against
  the same corpus, and worth ±1 dB on a fading channel when read as an
  absolute threshold. To average that down, generate several corpora with
  different `MFSK_SIM_SEED` values into separate out-dirs and compare.

So: **regenerating a corpus from the same simulator build reproduces it
exactly.** A baseline still assumes both machines built their simulators
from the same WSJT-X checkout — that part cannot be checked from inside
this repo.

**Every corpus carries a `.corpus-stamp`, and the sweep runner refuses one
without it** (since 2026-09-30, `scripts/lib/corpus-stamp.sh`). The
generators write it beside the WAVs: seed, simulator path and sha256, and
the commit that generated it. Two guards come with it. A generator refuses
a simulator that was not linked with `sim_sgran_stub.c` (the stub's
`MFSK_SIM_SEED` literal is what it looks for), and refuses to add cells to
a directory whose WAVs carry no stamp or another seed, since a generator
only fills in missing cells. `scripts/run-sensitivity-sweeps.sh` stops on a
corpus with no stamp or with the wrong seed, and notes one whose generator
or simulator build changed after it was stamped. `MFSK_SWEEP_ALLOW_UNSTAMPED=1`
overrides the stop; never `--update-baseline` from such a run.

What prompted it, on the 9900X box: a release sweep flagged JT65 +0.68 dB
worse in unchanged code. The local `jt65_sweep/`, `jt9_sweep/` and
`wspr_sweep/` were July/August corpora drawn from /dev/urandom, made before
the stub existed. Regenerated at seed 1, all four groups landed on the
baseline exactly. The same session found `target/ft8sim/ft8sim` and
`target/ft4sim/ft4sim`, the generators' default simulator paths, still
holding unseeded July/August builds. The corpora in use had come from the
seeded `target/ft8sim-2b9/` and `target/ft4sim-2b9/`; a generator run with
its defaults would have written an unreproducible corpus without a word.
Adopting the stamp meant regenerating every corpus into an empty directory
and comparing. All ten (`ft8`, `ft8_itu`, `ft8_busy`, `ft4`, `fst4`, `q65`,
`jt65`, `jt9`, `wspr`, `jtty`) came out byte-identical to the corpora they
replaced, 13 220 WAVs in all. The one difference was 90 `q65_sweep` files
that today's grid no longer writes: 5-trial cells at −19..−23 dB from an
earlier grid, on the 100 % plateau of their sub-modes. They are the surplus
behind the 2 820 against 2 640 trials above, and were left out.

#### 3.4.2 Background: the complete set

Counts and sizes as generated 2026-09-23 at each script's default
`TRIALS`; total ~17 GB.

| corpus | simulator | build via | WAVs | size |
|---|---|---|---|---|
| `fst4_sweep` | `fst4sim` | `scripts/build_fst4sim.sh` | 5120 | 13 G |
| `q65_sweep` | `q65sim` | WSJT-X CMake | 1320 | 2.0 G |
| `wspr_sweep` | `wsprsim` | `scripts/build_wsprsim.sh` | 260 | 716 M |
| `ft8_sweep` | `ft8sim` | `scripts/build_ft8sim.sh` | 1040 | 358 M |
| `jt9_sweep` | `jt9sim` | `scripts/build_jt9sim.sh` | 300 | 372 M |
| `jt65_sweep` | `jt65sim` | CMake + `scripts/build_jt65sim.sh` | 300 | 372 M |
| `ft4_sweep` | `ft4sim` | `scripts/build_ft4sim.sh` | 1040 | 147 M |

All seven regenerate byte-identically from the same simulator build and
`MFSK_SIM_SEED` — checked by running each generator twice and diffing the
WAVs, and end to end by rebuilding the whole 1040-file FT4 corpus and
finding it identical to the one on disk.

`q65_sweep`'s 120 s and 300 s sub-modes carry 5 trials per cell rather
than 15 — `gen_q65_sweep_wavs.sh` scales them down on purpose, since both
disk and decode cost grow with the T/R period.

Generating all seven is minutes of wall clock. *Running* the sweeps is
not: `fst4_sweep` over all five sub-modes was 8 h+ on the 3700X box
(~1100 audio-seconds per minute; on the Ryzen 9 9900X, 24 threads, the same
sweep through `scripts/run-sensitivity-sweeps.sh fst4` took 22 minutes on
2026-09-25, so re-measure before assuming the 8 h), which is what `MFSK_FST4_SWEEP_MODES`
/ `_CHANNELS` / `_SNR_MIN` / `_SNR_MAX` and `scripts/sweep-narrow-plan.py`
are for.

#### 3.4.3 The busy-band corpus

The AWGN/CCIR/ITU corpora hold one signal per file. `ft8_busy_sweep/` holds
several, scattered in time and frequency, plus files with none, because a
candidate-list cap, the lag window of the coarse sync and an acceptance gate that
admits garbage only show up there. `ft8sim` writes one signal per file, so
`scripts/gen_ft8_busy_wavs.py` asks it for each signal **without noise** (SNR 99),
rescales it to the SNR it should have, adds the signals and adds one realisation of
white Gaussian noise. No FT8 is re-implemented; every waveform is ft8sim's.

    scripts/gen_ft8_busy_wavs.py <ft8sim> embedded-poc/assets/ft8_busy_sweep    # 43 s, needs numpy

| set | files x signals | what a miss or an extra means |
|---|---|---|
| `dt1` | 100 x 1 | one signal at -16 dB, DT -0.5..+1.5 s: a miss is a time-window problem |
| `busy10`, `busy20`, `busy40` | 40 x 10 / 20 / 40 | 200-2700 Hz (12 Hz apart at least), DT -0.5..+1.5 s, SNR -24..-6 dB: crowding |
| `noise` | 200 x 0 | every decode is unexpected |

`truth.csv` lists every transmitted signal (`file,msg,f0,dt,snr`); 2900 rows, 2900
different messages. A truth signal is a **hit** when its message comes out within
5 Hz and 0.5 s of its true frequency and DT; an **extra** is a distinct decoded
message that was not transmitted in that file, by text. The signals are not faded
(the ITU corpus is for that), and DT stops at +1.5 s because ft8sim shifts the
waveform circularly inside its 15 s buffer.

**The SNR is calibrated against ft8sim, not assumed.** Fitting the unit waveform to
noisy files that ft8sim wrote itself gives amplitudes within 0.02 dB of
`sqrt(2*2500/6000) * 10^(SNR/20)` at -10 and -14 dB (ratio 0.999 and 0.998, eight
files each), and the noise standard deviation measured in the silence after the
signal is 1.001 (unit variance before the x100 gain). Regeneration is deterministic
(`--seed`, default 1: two runs are byte-identical); to check one:

    md5sum ft8_busy_busy20_05.wav   # e715b06cb83cce17eb5eb73fe1e8ccad
    md5sum ft8_busy_noise_33.wav    # ec465f7da97adff83224b0991d844aae
    md5sum truth.csv                # a02566306e684fcf6a6fe2c5f10e6a8e

Results: `BENCHMARKS.md`, "The busy-band corpus".

### 3.5 Generating the upstream baseline

```sh
scripts/upstream-baseline.py tasks                 # what is defined
scripts/upstream-baseline.py generate ft4/t1       # runs jt9 over ft4_sweep; seconds to minutes
```

Regenerate when the upstream tag changes or a corpus is regenerated.
`compare` refuses an upstream CSV whose corpus stamp does not match the
corpus on disk. All eight tasks take about 5 minutes on 24 threads, FST4 the longest at 2; JTTY's takes 3 s.

## 4. Running tier C

`scripts/run-sensitivity-sweeps.sh [protocol ...]` does, in order:

1. **Corpus checks.** A missing corpus is listed. One without a
   `.corpus-stamp` or with the wrong seed stops the run
   (`MFSK_SWEEP_ALLOW_UNSTAMPED=1` overrides; never update a baseline from
   such a run).
2. **The sweeps.** FT8, FT4 and FST4 sweep *as their upstream task T1*, so
   one run serves both checks. FST4 is narrowed to ±3 dB around each
   stored crossing (`scripts/sweep-narrow-plan.py`). `ft8_itu` and
   `ft8_busy` keep the library's default request, which keeps `decode()`'s
   single-pass path under a baseline. Per-trial CSVs go to
   `target/sweep-csv/`.
3. **Against its own past.** `scripts/sweep-regression-check.py` prints
   each group's 50 % crossing against `sweep-baseline.json`, flagging
   `!!` at ≥ 0.5 dB. For FT8, FT4 and FST4 it also prints unexpected
   decodes, flagging a rise of ≥ 3 and ≥ 1.5×.
4. **Against upstream.** For every task of a swept protocol,
   `scripts/upstream-baseline.py compare` pairs the CSV from step 2 with
   upstream's, and `time` times both sides on a few files per group.

The whole run, every corpus present, took 548 s on the Ryzen 9 9900X on
2026-10-01. It took 383 s before the upstream comparison was added. Of the
difference, about 100 s is the timing and the rest is the T1 sweeps' heavier
requests (FT8 at D3, FT4 with three subtraction rounds). Timing uses two files
per group and cell kind. That is enough for a ratio, but noisy for `wsprd`,
whose files take tens of milliseconds.

Nothing asserts. A flagged group is a question to answer before tagging.
Once the answer is understood, refresh the baseline with
`--update-baseline`, which stamps the date, commit, machine and trials for
the protocols in that run only. `scripts/release-status.sh` reads that
stamp to decide what is outstanding.

### 4.1 Reading the upstream table

```
group    trials both up only crate only     p   x up x crate  delta  extra up/crate
awgn        260  139       16          0 0.000 -18.17  -17.44  +0.72     0/0     !!
```

`up only` counts files upstream decoded and the crate did not; `crate only`
counts the reverse. `p` is the exact McNemar two-sided test on those two
counts. `!!` means upstream is significantly ahead, or the crate's
unexpected decodes exceed upstream's by more than 3. The crossings are
the usual interpolation, a summary only. The timing lines give ms per file
and the crate/upstream ratio, for noise-level files and files at upstream's
crossing.

#### 4.1.1 Background: pairing against `jt9`

A single corpus's 50%-crossing is not a statement about the decoder. The
draw it was generated from is worth up to ~1.1 dB on a fading channel, and
that error is *common to every decoder run against those files* — so the
way to remove it is to run WSJT-X's own binary over the same corpus and
read the difference, not the absolute number.

This is worth the two minutes it costs. On 2026-09-23 an FT8 corpus came
out 1.32 dB "worse" on `ccir_poor` than the previous measurement and
looked like a regression; the decoder was provably unchanged. Real
`jt9 -8 -d3` over the same files returned −18.88 dB against mfsk-core's
−18.90 dB — the corpus was simply a hard draw, and the eight-seed mean
(−19.68 dB) landed 0.01 dB from the figure the table had carried all
along.

Build the reference binary from the same CMake tree as `jt65sim`/`q65sim`:

```sh
cmake --build ~/wsjtx-build --target jt9 -j"$(nproc)"
# then, per WAV, in a scratch cwd:
~/wsjtx-build/jt9 -8 -d 3 -a . -t . <file>.wav     # -8 FT8, -5 FT4,
                                                   # -9 JT9, -6 JT65,
                                                   # -7 FST4, -3 Q65
```

Score its output with the sweep's own criteria (message match, frequency
within 5 Hz, |dt| ≤ 0.6 s) rather than a bare grep, and pick the depth
deliberately: `-d1`/`-d2`/`-d3` moved FT4's `ccir_poor` by 1.7 dB between
them, so a comparison that does not say which depth it used says very
little.

**Which `jt9` (decided 2026-09-26, #442).** The reference is built from the
**`v3.2.0-rc1` tag** of `WSJTX/wsjtx`, not from `2b9d654` and not from a moving
`master`: the FT8 and FT4 decoders this crate follows are that tag's (#439), and a
comparison against an older binary measures the port, not the decoder. Say the tag
(and the depth, and whether AP hits were counted) next to any `jt9` number.
`jt9` output has to be scored with its ` ?` / ` aN` markers stripped, and with the
AP decodes it prints at `-d 2`/`-d 3` even with no call sign given (`a1`, the CQ
pass) either counted or excluded on purpose: `scripts/score-jt9-sweep.py`.

Two things the tag does *not* replace. The **simulators** that generate the
`ccir_*` corpora stay on `2b9d654`: the Watterson fading model was corrected in
February 2024, so a newer `ft8sim` makes a harsher channel under the same name and
a baseline is only comparable within one tree (see "Which WSJT-X tree the
simulators come from"). And `jt9` numbers already recorded here from the `2b9d654`
build (the 2026-07/08 sections) stay as the dated records they are: they say what
that build gave on that day, and are not re-run. A `jt9` reference used *for a
decision* is the tag's.

#### 4.1.2 Background: why a task, and how it is scored

The paragraphs above describe a manual comparison. This one is the
standing version, and it rests on one idea: **define the task first**. A
comparison is only meaningful when both decoders are asked to do the same
thing. Q65's first comparison was not like that. `jt9 -3` tries the
Rx frequency (1500 ± 20 Hz) first, and the corpus signal sits exactly
there, while this crate scanned 200–3000 Hz with no Rx frequency. The same
comparison also ran AP as a separate full rescan. A task names the job
(band, what the operator knows, depth) and the configuration on each side
that does it. Accuracy and speed are compared within it.

- `scripts/upstream_tasks.json`: the tasks, each with the upstream command
  line and the crate request side by side. Edit both halves together.
- `scripts/build_jt9_upstream.sh`: builds `jt9` and `wsprd` from
  `v3.2.0-rc1` reproducibly, with the CMake edits this host needs. Check:
  `qso3_busy.wav` decodes 14 / 20 / 21 at `-8 -d1/-d2/-d3`.
- JTTY's upstream is `rjtty`, not `jt9`. `scripts/gen_jtty_sweep_wavs.sh`
  builds it beside `sjtty` from `v3.2.0-rc1` (`target/jttysim/build/rjtty`),
  and the task names that path (`upstream_path`). Its per-trial outcome must
  match the corpus's own `UPSTREAM_RECALL.tsv`, written by the generator
  with the same binary: it did, in all 18 cells.
- `scripts/upstream-baseline.py generate <task>`: runs upstream over the
  task's tier-C corpus once. It commits the per-trial outcome to
  `docs/notes/upstream/<task>.csv`, stamped with the binary's sha256, the
  flags and the corpus stamp. `compare` refuses a corpus that does not match.
- `scripts/upstream-baseline.py run <task> <dir>`: the crate half. It runs
  the sweep test with the task's configuration, pairs every trial with
  upstream's, and times both sides. `scripts/run-sensitivity-sweeps.sh`
  calls it for every task of a protocol it sweeps.

**Accuracy is paired, not compared as crossings.** Each group reports the
trials both decoded, those only upstream decoded, and those only the crate
decoded. It is flagged `!!` when upstream-only exceeds crate-only by an
exact McNemar test at p < 0.05. This is what lets the existing corpora
suffice. Two 20-trial crossings cannot resolve a few tenths of a dB; 260
paired trials can. So no corpus grows and CI is untouched. Unexpected
decodes are flagged when the crate's exceed upstream's by more than 3.

**Speed is a ratio on a few files.** Per channel, 5 files from the
lowest-SNR cell (effectively noise) and 5 from the cell nearest upstream's
crossing. Both sides decode one file at a time, single-threaded. Upstream's
figure is `timer.out`'s total, which leaves out process start, with FFTW
wisdom warm. The ratio is machine-independent enough to track.

## 5. Traps already stepped in

| symptom | cause | now |
|---|---|---|
| JT65 "+0.68 dB regression" in unchanged code | local corpus drawn from /dev/urandom before the seeded stub existed | corpus stamps; the runner refuses unstamped corpora |
| a generator writes an unreproducible corpus without a word | the default simulator path held an unseeded build | generators refuse a simulator without the stub |
| a 2× cost measured as "0–2 %" | two worktrees shared one `CARGO_TARGET_DIR` | principle 5 |
| a sweep "passes" in a second with no output | test binary run outside `cargo test`, no corpus found | row counts checked (principle 6) |
| rows doubled | sweep CSVs append | delete before reuse |
| `jt9` looks 1.5–2.5× slower than it is | fresh directory per file: process start and FFTW planning every time | `timer.out`, persistent directory |
| Q65 "at or above `jt9`" that measured nothing | different tasks: `jt9 -3` tries its Rx frequency first | task definitions |
| 0.4 dB invisible | 5-trial cells, crossing compared | pairing; more trials in scratch |
| a crossing "moved 1.3 dB" with the decoder unchanged | a new corpus draw on a fading channel | pair against `jt9` on the same files (§4.1.1) |
| `ccir_*` numbers do not match another machine's | simulators built from a post-February-2024 tree | build them from `2b9d654` (§3.1) |

## 6. Environment reference

| sweep test | corpus dir | CSV | task switch |
|---|---|---|---|
| `ft8_sweep` | `MFSK_FT8_SWEEP_DIR` | `MFSK_FT8_SWEEP_CSV` | `MFSK_FT8_SWEEP_TASK=t1` |
| `ft4_sweep` | `MFSK_FT4_SWEEP_DIR` | `MFSK_FT4_SWEEP_CSV` | `MFSK_FT4_SWEEP_TASK=t1` |
| `fst4_sweep` | `MFSK_FST4_SWEEP_DIR` | `MFSK_FST4_SWEEP_CSV` | `MFSK_FST4_SWEEP_TASK=t1`; also `_MODES`, `_CHANNELS`, `_SNR_MIN`, `_SNR_MAX` |
| `q65_sim_sweep` | `MFSK_Q65_SWEEP_DIR` | `MFSK_Q65_SWEEP_SUMMARY_CSV` | (the sweep is the task) |
| `jt9_sweep` | `MFSK_JT9_SWEEP_DIR` | `MFSK_JT9_SWEEP_SUMMARY_CSV` | (the sweep is the task) |
| `jt65_sweep` | `MFSK_JT65_SWEEP_DIR` | `MFSK_JT65_SWEEP_SUMMARY_CSV`, `MFSK_JT65_CHASE_SWEEP_SUMMARY_CSV` | (the Chase sweep is the task) |
| `wspr_sweep` | `MFSK_WSPR_SWEEP_DIR` | `MFSK_WSPR_SWEEP_SUMMARY_CSV` | (the sweep is the task) |
| `jtty_sweep` | `MFSK_JTTY_SWEEP_DIR` | `MFSK_JTTY_SWEEP_CSV` | (the sweep is the task: `Params::default()`) |

Also: `MFSK_SIM_SEED` (generators, default 1), `MFSK_SWEEP_FEATURES` and
`MFSK_SWEEP_FULL` (runner), `MFSK_SWEEP_ALLOW_UNSTAMPED`, `JOBS` and
`TRIALS` (generators).
