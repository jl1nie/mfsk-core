# m5stack-s3-app — agent notes

Production FT8 controller for M5StickS3 — LCD UI, QSO FSM,
WiFi UDP log streaming, ES8311 audio, planned USB UAC + BLE
CI-V to IC-705. The repo-root `CLAUDE.md` is authoritative
for flash-and-capture conventions (`scripts/flash-monitor.sh`,
USB-CDC freeze caveats); see also
[`embedded-poc/CLAUDE.md`](../CLAUDE.md) for the shared
Xtensa-LX7 toolchain notes and the LX6/LX7 comparison table.

## Crate-specific knobs

- **Target triple**: `xtensa-esp32s3-espidf`.
- **`opt-level = 1`** — same LX7 LLVM regression as the sibling
  `m5stack-s3` bench crate.
- **Partitions**: factory 3 MB + littlefs 1 MB. CSV path is
  derived from `CARGO_MANIFEST_DIR` (see commit `a03d9e7`).
- **sdkconfig**: BT NimBLE central + USB host stack enabled +
  Octal PSRAM. `SPIRAM_MALLOC_ALWAYSINTERNAL = 4096` (same as
  sibling crate; required for dual-core).

## Crate-specific gotchas

- **USB-CDC host disconnect freezes the FanoutLogger** —
  `println!` to USB-Serial-JTAG VFS blocks indefinitely after
  the host releases the CDC endpoint. Gate on
  `usb_serial_jtag_is_connected()`; see commit `8b46f4e` for
  the fix and `project_m5stick_s3_app.md` memory for the
  validation procedure.
- **`cfg.toml` (WiFi creds) is gitignored** — `cfg-sample.toml`
  is the committed template (wifikey2 convention).
- **WiFi UDP log sink target**: aterm-class APs drop subnet
  broadcast between stations; default `pc_ip = "auto"` may
  silently fail. Switch to unicast PC IP if heartbeat doesn't
  arrive.
- **WSL2 PC listener** needs a Hyper-V firewall rule for inbound
  UDP 9999 (mirrored-mode default blocks it); see
  `project_m5stick_s3_app.md` memory for the exact PowerShell
  invocation.

## Status (2026-05-14)

Phase 0 / 0.5 / 3 / 4 / 0.6 / 0.7 shipped on `main`. Next is
Phase 1 (USB UAC host capture from IC-705); after that
Phase 2 (BLE CI-V), Phase 5 (ADIF), Phase 6 (buttons), and
TX keying close the v0.7 transceiver-controller goal. See
`docs/ROADMAP.md` Phase B for the full phasing.
