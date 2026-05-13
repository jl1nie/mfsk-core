# m5stack-s3 — agent build/flash notes

M5StickS3 / ESP32-S3 (Xtensa LX7) compute bench. See
[`embedded-poc/CLAUDE.md`](../CLAUDE.md) for the shared
toolchain setup, flash-and-capture template, cross-board
debug list, and the LX6 vs LX7 comparison table — this file
only covers the crate-specific overrides.

## Crate-specific knobs

- **Target triple**: `xtensa-esp32s3-espidf`.
- **Bin name**: `mfsk-core-m5stack-s3` (with `rx_wavsim` as the
  primary streaming-pipeline driver bin).
- **Port**: S3 dev kits enumerate as `/dev/ttyACM0` (USB-Serial-JTAG,
  native S3) or `/dev/ttyUSB0` (CP210x bridge on some M5Stack S3
  modules). Confirm with `dmesg | tail` after plugging in.
- **PSRAM mode**: Octal (`CONFIG_SPIRAM_MODE_OCT=y`, ~80 MB/s) by
  default; Quad PSRAM boards (M5Stamp S3 etc.) switch to
  `_MODE_QUAD=y`.
- **`SPIRAM_MALLOC_ALWAYSINTERNAL = 4096`** is mandatory for
  dual-core dispatch. Bumping to `16384` puts both workers'
  `cs Box` (5 KB × 15 × 2) in internal DRAM and drains it →
  TLSF corruption → Guru Meditation.

## Crate-specific gotchas

- **dual-core dispatch + Phase E2 race** — historical LX6 issue
  carried over. `rx_wavsim` runs **sequential per-half on
  main**; the dispatch path is ported but not wired by default.
- **`release.opt-level`** must stay `1` (shared LLVM regression
  with Core2).
- **`tlsf_malloc` heap corruption** — same `decode_block`-direct-
  call bug as LX6; main.rs replicates the D pattern manually.

## Status (2026-05-14)

Bench-only crate; the live-RX scaffold (`rx_skeleton.rs`) was
retired in PR #66 (γ stage of `docs/CLEANUP_2026_05.md`). Live
RX + UI development lives in `embedded-poc/m5stack-s3-app/`.
