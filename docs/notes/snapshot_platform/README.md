# decode_snapshot across machines (#579)

`tests/decode_snapshot.rs` compares decode output with fixtures that pin f32 bit patterns,
frozen on x86_64. On other CPUs the decode is the same (same messages, same order) and
the last bits of the float columns are not. These files record how far.

| file | machine | rustc | fixtures differing | worst `snr_db` | worst `sync_score` (rel) | `hard_errors` |
|---|---|---|---|---|---|---|
| `aarch64-apple-m5-rustc1.98.1.txt` | Apple M5, macOS 27.2 | 1.98.1 (1.99.0 identical) | 50 of 53 | 0.038 dB | 1.9e-4 | +-1 in one FT8 row |
| `x86_64-ryzen7-3700x-rustc1.98.1.txt` | AMD Ryzen 7 3700X, WSL2 | 1.98.1 | 0 of 53 | 0 | 0 | 0 |
| `x86_64-ryzen7-7435hs-rustc1.99.0.txt` | AMD Ryzen 7 7435HS, CachyOS, glibc 2.44 (the reporter) | 1.99.0 | 4 of 53 | 2 ULP (4e-5 dB) | 0 | 0 |

## Measure another machine

```sh
MFSK_SNAPSHOT_REPORT=1 MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core \
    --features full,internal-testing --release --test decode_snapshot \
    -- --nocapture 2> snap.txt
python3 scripts/snapshot-diff-stats.py snap.txt
```

`MFSK_SNAPSHOT_REPORT=1` prints each mismatch (`SNAPDIFF <case>` blocks) and carries on, so
one run covers every case instead of stopping at the first. Unset, a mismatch fails as before.
`via_decoder::q65` has its own tolerance check further down the file and still stops at its
first failure; the other 13 tests run through.

To add the machine here, keep only the `SNAPDIFF` blocks (each case once), put the machine,
rustc and commit in `#` header lines as the existing file does, and name it
`<arch>-<cpu>-rustc<version>.txt`.

## Is it the libm? (`scripts/libm_probe.rs`)

The decoder's SNR goes through `f32::log10` and `f32::powf`, which call the platform libm
(glibc, Apple's, ...), not code in this repository. On x86_64 with AVX2 the FFT is the same
rustfft kernel on Zen 2 and Zen 3+, and rustfft 6.4.1 / realfft 3.5.0 have no AVX-512 code, so
a difference of one or two ULP in a few rows (the report in #579) points at the libm. The probe
tests that without the decoder: a fixed input list built from IEEE basic operations, one hash
per function. `sqrt` is a control and must match everywhere.

```sh
rustc --edition 2021 -O -o libm_probe scripts/libm_probe.rs
./libm_probe > libm_probe-<arch>-<cpu>.txt            # compare the hash column between machines
./libm_probe dump dumpA                               # on each machine, then copy one dir over
./libm_probe cmp dumpA dumpB                          # how many values differ, and by how many ULP
```

Different hashes for a function = that machine's libm gives other bits for it. Record `ldd --version`
(glibc) or the OS version beside the result. `libm_probe-aarch64-apple-m5.txt` is the first row.

### First comparison: Ryzen 7 3700X (glibc 2.35) against Apple M5

Measured, `libm_probe-x86_64-ryzen7-3700x.txt` vs `libm_probe-aarch64-apple-m5.txt`:

- `sqrt` (the control) has the same hash. The other seven functions (`log10`, `powf10`, `ln`,
  `exp`, `sin`, `cos`, `atan2`) all have different hashes: the two libms return different bits.
- Of the first three values per function, only `atan2`'s first differs (`bff5521c` vs `bff5521b`,
  1 ULP); the rest are equal. How many of the 1M values differ and by how many ULP needs a `dump`
  from the M5 (`libm_probe cmp`), not done yet.
- On this Ryzen, with that libm, `decode_snapshot` matches the fixtures in all 53 cases, under
  rustc 1.98.1 and 1.99.0 alike. So glibc 2.35 gives the fixtures' bits; Apple's libm does not.

Inference, not measured:

- A 1-ULP libm difference is ~1e-7 relative. The M5's worst `snr_db` is 0.038 dB and worst
  `sync_score` 1.9e-4 relative, three orders larger. If the libm is the cause it has to be
  amplified downstream (candidate ranking, a threshold, a subtraction pass); the probe cannot
  show that. aarch64 also runs a different rustfft kernel (NEON, not AVX2), a second candidate
  that this probe does not cover.
- The Ryzen 5 failure in the issue is x86_64 Linux, so it argues against "aarch64 only". Its
  glibc version and CPU generation would separate "libm version" from "FFT kernel dispatch".
  rustfft 6.4.1 / realfft 3.5.0 have no AVX-512 code, so AVX-512 is not a candidate on its own.

Where to set the tolerance: on Apple silicon, where the difference exists and can be iterated
on. x86_64 with this glibc needs none (0 differing fixtures). Suggested starting point from the
M5 numbers, to be checked there: messages and order exact, `freq_hz` / `dt_sec` exact,
`snr_db` <= 0.05 dB, `sync_score` <= 5e-4 relative, `hard_errors` +-1, `pass` exact.

### How much the libm differs: `cmp` of the two dumps (Apple M5 vs Ryzen 7 3700X, glibc 2.35)

`libm_probe cmp`, 1,000,000 values per function (the Ryzen dump was copied to the Mac):

| function | values that differ | largest gap |
|---|---|---|
| `log10` | 58,623 (5.9 %) | 2 ULP |
| `powf10` | 6,167 (0.6 %) | 1 ULP |
| `ln` | 959 (0.1 %) | 1 ULP |
| `exp` | 6,202 (0.6 %) | 1 ULP |
| `sin` | 145,079 (14.5 %) | 1 ULP |
| `cos` | 672 (0.07 %) | 1 ULP |
| `atan2` | 212,579 (21.3 %) | 2 ULP |
| `sqrt` (control) | 0 | 0 |

So the all-different hashes above are a few percent of the values off by one or two ULP, not a
wide gap. The SNR path's `log10` is off for 5.9 % of inputs. The M5's `snr_db` of 0.038 dB (about 0.9 % in the power ratio) and its many differing
`sync_score` values are far larger than a libm ULP can give, so aarch64 has at least one more
source (the NEON FFT kernel is the likely one). Neither is chased further: the tolerance below
is set from the measured size of the differences, not from their cause.

### The reporter's machine: Ryzen 7 7435HS, glibc 2.44, rustc 1.99.0

Measured by the reporter on `main` at `f6323dca` (#579). `decode_snapshot` passes with `Tol`, and
`MFSK_SNAPSHOT_REPORT=1` prints nothing. Under `MFSK_SNAPSHOT_STRICT=1` the same seven tests fail as
in the original report, from four rows in four fixtures (the report first named only the first;
the reporter corrected it):

| fixture | row | column | fixture → this machine |
|---|---|---|---|
| `ft8_default` | 4 | `snr_db` | -9.148531 → -9.148533 (2 ULP) |
| `ft4_default` | 5 | `freq_hz` | 1909.7092 → 1909.7094 |
| `q65_30a_averaged` | 0 | `snr_db` | -20.01653 → -20.016531 (1 ULP) |
| `iq_direct_ft8` | 14 | `snr_db` | -0.09790039 → -0.097904205 |

`dt_sec`, `sync_score`, `hard_errors`, `pass`, the messages and their order are exact everywhere.
That machine's `rustc` is Arch's (no rustup), so the pin is not honoured and it ran 1.99.0.

Its probe (`libm_probe-x86_64-ryzen7-7435hs.txt`) against the other two:

| function | glibc 2.44 equals glibc 2.35 | glibc 2.44 equals Apple's |
|---|---|---|
| `log10` | no | **yes** |
| `atan2` | no | no |
| `powf10`, `ln`, `exp`, `sin`, `cos` | yes | no |
| `sqrt` (control) | yes | yes |

Between glibc 2.35 and 2.44 only `log10f` and `atan2f` changed in what the probe covers, and
2.44's `log10f` now returns the same bits as Apple's. The `snr_db` rows fit the `log10f` change.
The FT4 `freq_hz` row is not traced: the `atan2` calls under `ft4/` and `engine/` are in tests,
and the run also changed the compiler (1.99.0), which made no difference on the M5 or the Zen 2.
Both stay well inside `Tol` (1.9e-4 Hz against 5e-3; 4e-5 dB against 0.1).

