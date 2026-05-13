# m5stack-core2 — agent build/flash notes

ESP32 Core2 (Xtensa LX6) compute bench. See
[`embedded-poc/CLAUDE.md`](../CLAUDE.md) for the shared
toolchain setup, flash-and-capture template, and cross-board
debug list — this file only covers the crate-specific
overrides.

## Crate-specific knobs

- **Target triple**: `xtensa-esp32-espidf`.
- **Bin name**: `mfsk-core-m5stack-core2`.
- **`.cargo/config.toml` runner**: `espflash flash --monitor`, so
  `cargo run --release` flashes and opens the serial monitor on
  an interactive TTY. For tee / pipe captures use
  `../scripts/flash-monitor.sh` as documented in the shared file.
- **Port**: M5Stack Core2 enumerates as `/dev/ttyACM0`. Pass
  `--port /dev/ttyACM0` explicitly so espflash never prompts.

## Crate-specific gotchas

- **`tlsf_malloc` heap corruption mid-sweep** — `decode_block`
  called directly from a long-running bench loop trips a known
  TLSF heap bug. Production `main.rs` works around it by inlining
  the per-iteration steps (the "D pattern") instead of calling
  `decode_block` as a single function.
- **`release.opt-level`** must stay `1`. `s` / `z` trigger an
  Xtensa-Rust LLVM regression on f32-select patterns; see the
  arithmetic-form workaround in `mfsk-core::core::pipeline` (FT4 SIC).

## Status (2026-05-14)

Bench-only crate; the live-RX scaffold (`rx_skeleton.rs`) was
retired in PR #66 (γ stage of `docs/CLEANUP_2026_05.md`). Core2
fold-in into the S3 dual-core pipeline is tracked under
[`#61`](https://github.com/jl1nie/mfsk-core/issues/61) — weekend
hardware bring-up.
