# m5stack-s3-app — agent notes

Production FT8 controller for M5StickS3 — LCD UI, QSO FSM,
WiFi UDP log streaming, ES8311 audio, planned USB UAC + BLE
CI-V to IC-705. The repo-root `CLAUDE.md` is authoritative
for flash-and-capture conventions (`scripts/flash-monitor.sh`,
USB-CDC freeze caveats); see also
[`embedded-poc/CLAUDE.md`](../CLAUDE.md) for the shared
Xtensa-LX7 toolchain notes and the LX6/LX7 comparison table.

**Operator manual** (build / flash / `cfg.toml` / `BootMode` cycle /
UI / QSO workflow / troubleshooting) lives at
[`docs/MANUAL_M5STICKS3.md`](../../docs/MANUAL_M5STICKS3.md). When
end-user behaviour changes (new boot mode, new menu, new troubleshooting
entry) update that file, not this one.

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

## Status (2026-05-21 — demo / acoustic-fallback, frozen after Phase 1.7)

Phase 0 / 0.5 / 3 / 4 / 0.6 / 0.7 shipped on `main`. Phase 1 UAC
hardware verification confirmed **M5StickS3 cannot do USB host**
(board lacks VBUS source circuit, ID pin wiring, host power switch
IC; see memory `project_m5stick_s3_no_usb_host`). This crate is
repositioned as the **demo / acoustic-fallback** path:

- **Phase 1.5 (Acoustic) — ✅ done**: ES8311 ADC mic-mode → I2S RX →
  `LinearResamplerI16To12k` → `decode_pipeline::run_with_source`.
  `BootMode::Acoustic` added to the NVS cycle. Live on `main`.
- **Phase 1.7 (QSO bidir I2S + demo toggle) — ✅ done**: BtnA
  long-press in `BootMode::Qso` toggles `demo_mode_enabled`; the
  audio thread substitutes `qso3_busy.wav` for both the decoder's
  chunk feed and the speaker output. On `feat/demo-mode-qso`, PR #121.
- **Phase 1.7.8 (NextMode menu item) — ✅ done**: 4th menu item
  lets the user switch boot mode from within the menu overlay (no
  3-reboot KEY dance). On `feat/demo-mode-qso` (same PR #121 branch).
- **Phase 1.7.9 (cold-start auto-sync bootstrap) — ✅ done**: PR
  #133 / v0.6.6. `decode_pipeline.rs` auto-sync gains a fourth
  branch — when `n_dec == 0 && best_n == 0 && bootstrap_dt_med
  .is_some()`, coarse_sync top-5 DT median drives a one-shot
  slot shift while `best_n` stays at 0 (soft anchor; first
  confirmed-decode slot reclaims HWM via the existing path).
  Removes the "BtnA required" cold-start dead-end on quiet
  bands. Helper lives at `mfsk_core::core::sync::bootstrap_dt_median`
  and is shared with WebFT8.
- **Phase 1 (UAC), Phase 2 (BLE CI-V), Phase 5 (ADIF), Phase 6
  (buttons), TX keying**: rolled forward to `m5stack-cores3-app`
  (Phase B-Core in `docs/ROADMAP.md`). **Stick frozen after 1.7.9 —
  1.7.9 is a controller-side auto-sync delta, no new HW surface.**

The `uac.rs` module (~445 lines) stays in-tree as canonical
reference; it's cloned verbatim into `m5stack-cores3-app` in Phase
0-Core, then hoisted into `mfsk-app-shared` in Phase 1.5-Core after
dual-board verification.
