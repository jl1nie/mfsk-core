# Changelog

## 0.1.0 — 2026-04-19

Initial release. Consolidates nine previously-separate workspace
crates from the `jl1nie/webft8` project into a single `mfsk-core`
crate with feature-gated protocol modules:

- `mfsk-core`, `mfsk-fec`, `mfsk-msg` → `core`, `fec`, `msg` modules
- `ft8-core`, `ft4-core`, `fst4-core`, `wspr-core`, `jt9-core`,
  `jt65-core` → per-protocol modules behind features of the same
  name

Features shipped at 0.1.0:

- FT8 (15 s, 8-GFSK, LDPC(174, 91))
- FT4 (7.5 s, 4-GFSK, LDPC(174, 91))
- FST4-60A (60 s, 4-GFSK, LDPC(240, 101))
- WSPR (120 s, 4-FSK, convolutional r=½ K=32 + Fano, incl. Type 1/2/3)
- JT9 (60 s, 9-FSK, convolutional r=½ K=32 + Fano)
- JT65 (60 s, 65-FSK, RS(63, 12) GF(2⁶), incl. erasure-aware decode)

Companion (not published): `mfsk-ffi` sibling crate exposing a
C ABI + `mfsk.h` header via cbindgen, with C++ driver and Kotlin
JNI example scaffolds.

Algorithms derived from WSJT-X (K1JT et al.); each source file
cites the corresponding upstream file.
