# decode_snapshot across machines (#579)

`tests/decode_snapshot.rs` compares decode output with fixtures that pin f32 bit patterns,
frozen on x86_64. On other CPUs the decode is the same (same messages, same order) and
the last bits of the float columns are not. These files record how far.

| file | machine | rustc | fixtures differing | worst `snr_db` | worst `sync_score` (rel) | `hard_errors` |
|---|---|---|---|---|---|---|
| `aarch64-apple-m5-rustc1.98.1.txt` | Apple M5, macOS 27.2 | 1.98.1 (1.99.0 identical) | 50 of 53 | 0.038 dB | 1.9e-4 | +-1 in one FT8 row |
| `x86_64-ryzen7-3700x-rustc1.98.1.txt` | AMD Ryzen 7 3700X, WSL2 | 1.98.1 | 0 of 53 | 0 | 0 | 0 |

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
a difference that is only `snr_db` by one ULP (the report in #579) points at the libm. The probe
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
