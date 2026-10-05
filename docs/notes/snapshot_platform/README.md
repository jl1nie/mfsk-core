# decode_snapshot across machines (#579)

`tests/decode_snapshot.rs` compares decode output with fixtures that pin f32 bit patterns,
frozen on x86_64. On other CPUs the decode is the same (same messages, same order) and
the last bits of the float columns are not. These files record how far.

| file | machine | rustc | fixtures differing | worst `snr_db` | worst `sync_score` (rel) | `hard_errors` |
|---|---|---|---|---|---|---|
| `aarch64-apple-m5-rustc1.98.1.txt` | Apple M5, macOS 27.2 | 1.98.1 (1.99.0 identical) | 50 of 53 | 0.038 dB | 1.9e-4 | +-1 in one FT8 row |

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
