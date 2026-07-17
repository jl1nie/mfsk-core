# mfsk-core M5Stack S3 compute bench

Decoder-only timing-regression bench for **ESP32-S3 (Xtensa LX7
dual-core @ 240 MHz, 8 MB Octal PSRAM)**. Drives
`mfsk_core::ft8::decode_block` against the vendored WSJT-X reference
WAVs in `../assets/` (`qso1`, `qso2`, `qso3_busy`) and prints
per-stage timing for each slot. Not for end users — for end users
see [`docs/reference/MANUAL_M5STICKS3.md`](../../docs/reference/MANUAL_M5STICKS3.md).

The sibling LX6 bench (`embedded-poc/m5stack-core2/`) was retired
in PR #76 (#61 Phase 3, 2026-05-16); the Core2 wall-clock path is
now exercised by the production-app crate
[`embedded-poc/m5stack-core2-app/`](../m5stack-core2-app/) running
its `wav_sim` decoder loop. This S3 bench remains because the S3
optimization sweeps (Goertzel migration, work-stealing dispatch,
Phase-E stage1_inc) needed a pure-decoder driver without the
controller-shape overhead.

## Build & flash

See [`embedded-poc/CLAUDE.md`](../CLAUDE.md) for the shared
toolchain setup (`espup install`, `~/export-esp.sh`). Crate-specific
overrides (target triple, port, PSRAM mode, opt-level constraint)
live in [`CLAUDE.md`](CLAUDE.md). Quick path:

```sh
source ~/export-esp.sh
cd embedded-poc/m5stack-s3
cargo build --release --bin rx-wavsim
../scripts/flash-monitor.sh \
    target/xtensa-esp32s3-espidf/release/rx-wavsim \
    logs/s3_$(date +%Y-%m-%d).log \
    90
```

The bench prints per-stage timing in the form recognised by
`docs/reference/EMBEDDED.md`'s performance benchmark table; cross-reference
that doc for the canonical 0.6.x post-SlotEnd numbers.

## Status

S3 LX7 compute bench. The live-RX scaffold (`rx_skeleton.rs`) was
retired in PR #66 (γ stage of `docs/CLEANUP_2026_05.md`). All live
RX + UI development happens in
[`embedded-poc/m5stack-s3-app/`](../m5stack-s3-app/).
