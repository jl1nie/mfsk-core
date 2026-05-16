# m5stack-core2-app — agent notes

M5Stack Core2 (ESP32 LX6) FT8 controller. Sibling of `m5stack-s3-app`
sharing `mfsk-app-shared` for board-agnostic QSO / UI / WiFi /
log fanout. See `../CLAUDE.md` for the shared embedded toolchain
setup and LX6/LX7 comparison table; this file covers Core2-specific
deltas only.

## Crate-specific knobs

- **Target triple**: `xtensa-esp32-espidf` (LX6).
- **`opt-level = 1`**: mirror of the sibling crates' LLVM regression
  workaround.
- **PSRAM**: `SPIRAM_MODE_QUAD=y` (Core2 wiring, ~4 MB usable).
- **`SPIRAM_MALLOC_ALWAYSINTERNAL = 4096`**: mandatory for dual-core
  decode (same lesson as both sibling crates).
- **Partitions**: factory 3 MB + littlefs 1 MB on a 16 MB flash; the
  remainder is unused for now. Partition CSV path is anchored via
  `sdkconfig.gen.defaults` (run `./bootstrap-sdkconfig.sh` once on
  fresh clone — see sibling `m5stack-s3-app` for rationale).
- **No USB host**: classic ESP32 has no USB-OTG hardware; the IC-705
  UAC capture path is S3-only.
- **No BLE**: deferred to Phase 2.5+, same as the S3 sibling.
- **`mfsk-app-shared` with `default-features = false`**: disables the
  `usb-serial-jtag` feature, since Core2 uses an external CH9102F /
  CP2104 USB-UART bridge with no equivalent connect-probe API.
  `log_sink::FanoutLogger` falls back to unconditional UART println
  — failure mode is different from S3 (no host-side TX-FIFO block
  observed in Phase 2 testing, but if it appears, gate via a new
  Core2-specific feature).

## Phase 2 scope (2026-05-16, this PR)

Minimum-viable Core2 variant of the S3 app, focused on proving the
shared crate boundary works for a second consumer:

- **Audio**: `embedded_shared::wav_sim` (`qso3_busy.wav` loop). PDM
  mic bring-up + NS4168 speaker out are Phase 2.5+ items — the memory
  note `project_streaming_api.md` flags Core2 PDM as UNVERIFIED, so
  scaffolding it Phase 2 was deemed too risky for the carve-out PR.
- **Input**: none. Core2 has no GPIO buttons (3 touch zones via
  FT6336U + AXP192 power button only). `boot_mode::determine_no_override`
  reads the NVS-stored mode directly; flipping requires a host-side
  NVS edit until Phase 2.5 adds touch.
- **Display**: ILI9342C 320×240 landscape, rendered via mipidsi's
  ILI9341 model. The shared `mfsk_app_shared::ui::*` draw routines
  hardcode 135 px width (M5StickS3 panel); Phase 2 renders them into
  the **top-left 135×240 corner** of the Core2 panel, leaving the
  right ~185 px black. Widening to a runtime canvas dim is a
  Phase 2.5 item.

## Crate-specific gotchas

- **AXP192 LCD power sequence**: LDO2 = 3.3 V (LCD VDD + backlight),
  DCDC3 = 2.8 V (LCD logic), then AXP192 GPIO4 LO→HI to release the
  panel's reset. `pmic::init_lcd_power` runs this before any SPI
  traffic; skipping it leaves the panel dark even with a perfect
  SPI bring-up. (Same lesson the sibling crate learned with M5PM1.)
- **SPI baud = 20 MHz** for the LCD on first bring-up. The maiden
  flash with 40 MHz hung the render loop after `tick=0` (no panic,
  silent stall). Bump to 40 MHz only after a Phase 2 baseline
  capture confirms steady-state behaviour.
- **mipidsi `display_size` must match `FRAMEBUFFER_SIZE`**
  (240, 320) — the panel's native portrait. Pass
  `Orientation::new().rotate(Rotation::Deg90)` to rotate the
  embedded-graphics coordinate space to landscape 320×240. Passing
  `display_size(320, 240)` directly trips an internal assert in
  mipidsi 0.8 (`builder.rs:157`).
- **Flash time vs monitor capture**: a fully changed 1.3 MB Core2
  binary takes ~55 s to flash. Use ≥120 s for the
  `scripts/flash-monitor.sh` capture window, otherwise the monitor
  starts after the chip has already booted and you miss the early
  log lines.
- **Internal DRAM is tight** (~17–20 KB free, 13–16 KB largest after
  BASIS alloc + LCD SPI driver). Phase 2 runs are stable but the
  margin is narrow; any future internal-only allocation needs to
  account for this. Compare to the sibling S3 which has ~185 KB
  internal free in the same configuration.

## Status (2026-05-16)

- Phase 2 boot + LCD render + decode loop verified on the user's
  Core2 hardware (commit on `feat/61-mfsk-app-shared`).
- 7/8 decodes per slot from `qso3_busy.wav` — matches the S3 sibling's
  recall. Internal-DRAM headroom is the binding constraint for adding
  features (touch / WiFi simultaneous run).
- Next (Phase 3 of issue #61): retire the now-redundant
  `embedded-poc/m5stack-core2/` bench crate.
