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
  the fix.
- **`cfg.toml` (WiFi creds) is gitignored** — `cfg-sample.toml`
  is the committed template (wifikey2 convention).
- **WiFi UDP log sink target**: aterm-class APs drop subnet
  broadcast between stations; default `pc_ip = "auto"` may
  silently fail. Switch to unicast PC IP if heartbeat doesn't
  arrive.
- **WSL2 PC listener** needs a Hyper-V firewall rule for inbound
  UDP 9999 (mirrored-mode default blocks it). The exact
  `New-NetFirewallHyperVRule` invocation is captured in the
  Phase 0.6 commit log (search `git log --grep "Phase 0.6"`).

## Status (2026-05-17 pivot — demo / acoustic-fallback role)

Phase 0 / 0.5 / 3 / 4 / 0.6 / 0.7 shipped on `main`. Phase 1 UAC
hardware verification confirmed **M5StickS3 cannot do USB host**
(board lacks VBUS source circuit, ID pin wiring, host power switch
IC; see memory `project_m5stick_s3_no_usb_host`). This crate is
repositioned as the **demo / acoustic-fallback** path:

- **Phase 1.5 (next, task #47)**: internal MEMS mic via ES8311 ADC
  mode → I2S RX → `decode_pipeline::run_with_source`. Picks up
  IC-705 SPEAKER OUT acoustically; no cable / dongle required. Adds
  `BootMode::Acoustic` between `Wifi` and `Uac` in the boot cycle.
- **Phase 1 (UAC), Phase 2 (BLE CI-V), Phase 5 (ADIF), Phase 6
  (buttons), TX keying**: rolled forward to `m5stack-cores3-app`
  (Phase B-Core in `docs/ROADMAP.md`).

The `uac.rs` module (~445 lines) stays in-tree as canonical
reference; it's cloned verbatim into `m5stack-cores3-app` in Phase
0-Core, then hoisted into `mfsk-app-shared` in Phase 1.5-Core after
dual-board verification.
