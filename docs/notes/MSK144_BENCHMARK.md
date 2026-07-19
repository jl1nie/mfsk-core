# MSK144 sensitivity benchmark — environment setup

How the MSK144 SNR sensitivity sweep (`tests/msk144_snr_sweep.rs`) was
verified against real WSJT-X, and how to reproduce that verification
from a clean checkout. See
[#156](https://github.com/jl1nie/mfsk-core/issues/156) for the
investigation this grew out of (a follow-up to the SNR-bias fix in
[#157](https://github.com/jl1nie/mfsk-core/pull/157)).

Unlike the FT4/FST4 benchmark harnesses
([`FT4_BENCHMARK.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/FT4_BENCHMARK.md),
[`FST4_BENCHMARK.md`](https://github.com/jl1nie/mfsk-core/blob/main/docs/notes/FST4_BENCHMARK.md)),
which need a small standalone Fortran `*sim` binary built from just
`lib/`, **MSK144's day-to-day regression test needs no WSJT-X checkout
at all** — `tests/msk144_snr_sweep.rs` is fully self-contained (its
own synthetic-signal generator, its own noise model, no external
process). The WSJT-X-comparison step described here (building the
full `jt9` CLI) was a **one-time verification**, not something CI or
routine development needs to repeat. Reproduce it only if you want to
re-check against a newer WSJT-X release or extend the SNR grid.

## 1. The regression test (no WSJT-X needed)

```sh
cargo test --release -p mfsk-core --features "msk144,fft-rustfft,uvpacket,parallel" \
  --test msk144_snr_sweep -- --ignored --nocapture
```

- `--release` and `--features parallel` both matter a lot here: the
  sweep is 2 ping-length configs × 7 SNR points × 20 seeds, each seed
  a full `Depth::Deep` `decode_slot()` call over a 15-30 s slot.
  ~266 s in release without `parallel` (the per-seed trials are
  independent, sequential otherwise), ~22 s with it (rayon
  `into_par_iter()` over the 20 seeds per SNR point).
- Output is a plain recall table (hits/20 per SNR point) — no
  pass/fail assertion. Like the FT4/FST4/FT8 sweep tests, a hardcoded
  threshold-dB gate isn't a stable regression signal here (sensitive
  to FFT backend / floating-point details that don't reflect a real
  recall regression); watch the printed table by eye against the
  baseline below.
- CI runs this automatically post-merge to `main` via the "catchall
  characterization" suite (`.github/workflows/ci.yml`) — add the
  `run-full-sweep` label to a PR to force it there too.

The synthesizer inside the test reproduces WSJT-X's own `msk144sim.f90`
signal model directly in Rust (not this crate's own OQPSK TX path, so
recall numbers aren't self-fulfilling against this crate's own
modulator):

- A meteor ping once per second across the slot
  (`t = 1, 2, ..., ntr_period - 1`), `2.718 * t * exp(-t)` decay
  envelope (`makepings.f90`).
- Continuous-phase binary FSK carrier, recovered from this crate's own
  LDPC(128,90) + CRC-13 encoder output via the differential relation
  `msk144::decode`'s own independent-oracle unit test already uses —
  i.e. the *message encoding* is this crate's own, but the *waveform
  synthesis* deliberately is not.
- AWGN calibrated to WSJT-X's **2500 Hz reference bandwidth** SNR
  convention (see below) — so the `snr_db` values printed by the sweep
  are directly comparable to WSJT-X's own reported SNR, not this
  crate's raw full-Nyquist-band convention.

## 2. Reproducing the one-time WSJT-X cross-check

### 2.1 Prerequisites

MSK144's `jt9` target pulls in WSJT-X's *entire* dependency chain at
CMake configure time (`find_package(... REQUIRED)` for all of these
runs unconditionally, regardless of which target you actually build)
— this is a much heavier prerequisite set than FT4/FST4's standalone
`*sim` binaries:

| Requirement | Ubuntu/Debian package | Purpose |
|---|---|---|
| Qt5 (Widgets, SerialPort, Multimedia, PrintSupport, Sql, LinguistTools) | `qtbase5-dev qtmultimedia5-dev libqt5serialport5-dev qttools5-dev qttools5-dev-tools` | linked by `fort_qt` |
| Boost (log, log_setup) | `libboost-log-dev libboost-dev` | `find_package(Boost ... log_setup log)` |
| libusb-1.0 | `libusb-1.0-0-dev` | Hamlib's optional USB rig backends |
| readline | `libreadline-dev` | Hamlib CLI history |
| autoconf/automake/libtool/pkg-config | same names | Hamlib's `./bootstrap` |
| `cmake`, `gfortran`, `libfftw3-dev` | same names | usually already present |

```sh
sudo apt-get install -y \
  qtbase5-dev qtmultimedia5-dev libqt5serialport5-dev \
  qttools5-dev qttools5-dev-tools \
  libboost-log-dev libboost-dev \
  libusb-1.0-0-dev libreadline-dev \
  autoconf automake libtool pkg-config git
```

### 2.2 Build Hamlib from source

`WSJT-X/INSTALL`'s own recipe calls for Hamlib's `integration`
branch — **that branch no longer exists** (merged into `master` some
time ago); use `master` instead, everything else in the recipe still
applies:

```sh
mkdir -p ~/hamlib-prefix && cd ~/hamlib-prefix
git clone https://github.com/Hamlib/Hamlib src
cd src && ./bootstrap
mkdir ../build && cd ../build
../src/configure --prefix=$HOME/hamlib-prefix \
   --disable-shared --enable-static \
   --without-cxx-binding --disable-winradio \
   CFLAGS="-g -O2 -fdata-sections -ffunction-sections" \
   LDFLAGS="-Wl,--gc-sections"
make -j"$(nproc)" && make install-strip
```

### 2.3 Build `jt9` and `msk144sim`

```sh
mkdir -p ~/wsjtx-build && cd ~/wsjtx-build
cmake -D CMAKE_PREFIX_PATH=$HOME/hamlib-prefix \
  -DWSJT_SKIP_MANPAGES=ON -DWSJT_GENERATE_DOCS=OFF \
  /path/to/WSJT-X
cmake --build . --target jt9 -j"$(nproc)"
cmake --build . --target msk144sim -j"$(nproc)"
```

`--target jt9`/`--target msk144sim` skip compiling the full Qt GUI
application (`wsjtx` itself) — CMake still configures everything
(hence the full dependency list above), but the actual build stays
scoped to the two binaries needed here.

**Sanity check** against the vendored golden WAVs before trusting the
build for anything else:

```sh
cd /path/to/WSJT-X/samples/MSK144
~/wsjtx-build/jt9 -k -d 3 -f 1477 -F 60 181211_120500.wav 181211_120800.wav
```

Should reproduce all 3 golden decodes from
[`reference_msk144_jt65_wsjtx_sample_decode.md`](https://github.com/jl1nie/mfsk-core)-equivalent
(`mfsk-core/tests/msk144_wsjtx_samples.rs`'s `Golden` table) —
message, frequency, timing, and SNR all matching. The default `-F 20`
frequency-search tolerance is too narrow for the 1458 Hz signal in the
second file; widen it as shown.

**Foot-gun**: `jt9` drops `decoded.txt` / `jt9_wisdom.dat` /
`timer.out` into whatever the current directory was when it ran — run
it from a scratch directory, not the repo root.

### 2.4 `msk144sim`'s SNR convention

`msk144sim.f90` scales its injected noise by
`fac = sqrt(6000.0/2500.0)` before adding it — its `snrdb` argument is
calibrated to WSJT-X's **2500 Hz reference bandwidth**, even though
the simulation itself runs at 12 kHz sample rate / 6 kHz Nyquist. This
is the authoritative source for the "+10·log10(6000/2500) ≈ +3.8 dB"
conversion between this crate's raw full-Nyquist-band SNR convention
(used elsewhere, e.g. `msk144::decode::SlotDecode::snr_db`'s own
internal `pmax`/`pnoise` computation) and WSJT-X's reported figures —
confirmed directly from the simulator source, not inferred.

**Gotcha**: `msk144sim`'s RNG is *not* re-seeded across separate
process invocations — running it twice with identical arguments
produces byte-identical WAVs. Independent noise realizations only come
from a *single* invocation with `nfiles > 1` (the RNG stream advances
across the internal `do ifile=1,nfiles` loop).

```
Usage:   msk144sim       message      TRp freq width snr nfiles
Example: msk144sim "K1ABC W9XYZ EN37"  15 1500  0.12   2    1   # short ping
         msk144sim "K1ABC W9XYZ EN37"  30 1500  2.5   15    1   # long ping
```

`TRp`/`width` set the meteor-ping profile: `TRp=15, width=0.12` gives
a ~0.4 s ping once/second across a 15 s slot; `TRp=30, width=2.5`
gives a ~2.5 s ping once/second across a 30 s slot. These are
WSJT-X's own worked examples and are what `tests/msk144_snr_sweep.rs`
reproduces as its "short"/"long" configs.

### 2.5 Running the cross-check

There's no checked-in script for this (it was a one-time
verification, not a repeatable CI step) — the shape of it:

```sh
for snr in -9 -8 -7 -6 -5 -4 -3; do
  msk144sim "K1ABC W9XYZ EN37" 15 1500 0.12 "$snr" 20   # one call, 20 files
  for f in 000000_*.wav; do
    jt9 -k -d 3 "$f" | grep -qF "K1ABC W9XYZ EN37" && echo hit
  done
done
```

...decoding the same WAVs with both `jt9 -k -d 3` and a throwaway Rust
binary calling `msk144::decode::decode_slot()` directly (a small
`path`-dependency Cargo project against the local `mfsk-core` checkout
is enough — no need to add a permanent binary target to the crate).

## 3. Results (2026-07-19, vs WSJT-X `jt9`)

20 seeds/SNR point, both configs:

| config | SNR (dB) | jt9 | mfsk-core |
|---|---|---|---|
| short (~0.4 s ping) | -7 | 0/20 | 0/20 |
| | -6 | 3/20 | 3/20 |
| | -5 | 17/20 | 16/20 |
| | -4 | 20/20 | 20/20 |
| long (~2.5 s ping) | -7 | 1/20 | 0/20 |
| | -6 | 1/20 | 1/20 |
| | -5 | 14/20 | 13/20 |
| | -4 | 20/20 | 20/20 |

**25 of 28 cells matched exactly; the other 3 differed by exactly 1
file out of 20** — a single borderline noise realization at the
steepest part of the threshold curve, not a systematic gap. 50%
recall crossing ≈ -5.5 to -6 dB (WSJT-X's 2500 Hz reference-bandwidth
convention) for both ping profiles.

This is the baseline to compare future `tests/msk144_snr_sweep.rs`
runs against — its own synthesizer isn't byte-identical to
`msk144sim`'s (independent Rust reimplementation of the same model,
own RNG), so expect the printed numbers to be *close* to this table
rather than identical to it; a multi-dB shift from this baseline is
the signal worth investigating, not small per-cell noise.
