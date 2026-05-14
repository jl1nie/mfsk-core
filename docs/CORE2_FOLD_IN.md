# Core2 fold-in into the production controller (issue #61)

**Status (2026-05-15):** plan ratified, ready for Phase 1 (lib
extraction). Phases 3-5 are the weekend hardware bring-up window
(2026-05-17/18). ε `decode_block` restructure (`docs/CLEANUP_2026_05.md`)
is intentionally sequenced **after** Phase 5 so the two refactors do
not collide.

## Context

The embedded port currently maintains two parallel decode bench crates
(`embedded-poc/m5stack-core2`, `embedded-poc/m5stack-s3`) plus the
production controller (`embedded-poc/m5stack-s3-app`). The bench shims
already collapsed into `embedded-poc/embedded-shared/src/apps/` — the
remaining duplication is **infrastructure** (Cargo.toml, sdkconfig,
`.cargo/config.toml`, partitions) and **board peripheral glue** that
exists only in S3 shape under `m5stack-s3-app/`. The LX6 path has no
in-tree LCD / PMIC / button / audio driver; the old `rx_skeleton.rs`
PDM scaffold was pseudocode only and was retired in PR #66.

Goal: bring Core2 (LX6) onto the same production-shape codebase as the
S3 controller so the LX6 board is a first-class decode target rather
than a separate bench crate. Tracked under
[#61](https://github.com/jl1nie/mfsk-core/issues/61).

## Architectural constraint that shapes the plan

Issue #61 lists as a **non-goal**: *"Single-binary that flashes to
either board without a target switch — the toolchain still picks
`xtensa-esp32-espidf` vs `xtensa-esp32s3-espidf` at compile time."*

Empirically this is **forced** by `esp-idf-sys`:

- `.cargo/config.toml` `target = "..."` is per-invocation, not
  per-feature; Cargo cannot feature-gate the default target.
- `[package.metadata.esp-idf-sys].esp_idf_sdkconfig_defaults` is a
  build-script-time literal — it cannot select among sdkconfig files
  via feature flags.
- The MCU env var (`MCU=esp32` vs `MCU=esp32s3`) routed through
  `.cargo/config.toml` is similarly per-crate.

Therefore the unified codebase **must** be a **library crate** that two
**thin binary crates** depend on:

```
embedded-poc/
├── mfsk-app-shared/        ← [lib] crate, holds the production-shape code
├── m5stack-s3-app/         ← [bin] shim, target = xtensa-esp32s3-espidf
└── m5stack-core2-app/      ← [bin] shim, target = xtensa-esp32-espidf  (new)
```

This honours #61's "no single-binary" non-goal while collapsing the
real duplication (board glue + production logic). Per-shim
`Cargo.toml` + `sdkconfig.defaults` is ~70 lines of config;
`mfsk-app-shared` is what holds the 20-file production codebase.

## Critical files to touch

Library extraction (Phase 1):

- All 20 files under `embedded-poc/m5stack-s3-app/src/` move to
  `embedded-poc/mfsk-app-shared/src/`. Module tree preserved; only
  the `main.rs` entry function becomes `pub fn run<B: Board>() -> !`.
- `embedded-poc/m5stack-s3-app/src/main.rs` shrinks to ~10 lines
  (link patches + `mfsk_app_shared::run::<board::BoardS3>()`).
- `embedded-poc/m5stack-s3-app/Cargo.toml` adds `mfsk-app-shared`
  path dep, drops the body deps that move into the library.

Board trait (Phase 2):

- New `mfsk-app-shared/src/board.rs` defines the `Board` trait.
  Replaces the current `m5stack-s3-app/src/board.rs` constants-only
  file. Modules that take board-specific types as parameters today
  (display.rs, pmic.rs, audio.rs, buttons.rs) become generic over `B`.
- New `mfsk-app-shared/src/boards/s3.rs` — `BoardS3 impl Board` plus
  the S3-specific ST7789P3 / M5PM1 / ES8311 init that currently lives
  inline in display.rs / pmic.rs / audio.rs.
- New `mfsk-app-shared/src/boards/core2.rs` — `BoardCore2 impl Board`
  (Phase 3 fills this in).

Core2 board impl (Phase 3):

- `mfsk-app-shared/src/boards/core2/lcd.rs` — ILI9342C init via
  `mipidsi` crate (already supports both ST7789 and ILI9342C — same
  pattern as the existing S3 display driver).
- `mfsk-app-shared/src/boards/core2/pmic_axp192.rs` — AXP192 init
  (I2C 0x34): LCD backlight rail, LDO2/LDO3 power gates, button
  charge state.
- `mfsk-app-shared/src/boards/core2/audio_pdm.rs` — SPM1423 PDM mic
  capture (GPIO0 = SCK, GPIO34 = DIN, mono 16 kHz → 12 kHz
  resampler already lives in `embedded-shared::wav_sim`-style Q32
  linear). New code; rx_skeleton's pseudocode is a starting outline
  only.
- `mfsk-app-shared/src/boards/core2/buttons.rs` — BtnA / BtnB / BtnC
  GPIO39/38/37 polling (mirror existing buttons.rs shape).

Core2 binary crate (Phase 4):

- `embedded-poc/m5stack-core2-app/Cargo.toml` — references
  `mfsk-app-shared` via path, target = xtensa-esp32-espidf,
  QUAD PSRAM features.
- `embedded-poc/m5stack-core2-app/.cargo/config.toml` — runner
  espflash, port `/dev/ttyACM0`.
- `embedded-poc/m5stack-core2-app/sdkconfig.defaults` — Core2-tuned
  (QUAD PSRAM, no BT, optional WiFi).
- `embedded-poc/m5stack-core2-app/partitions.csv` — Core2 16MB
  flash layout (factory 3MB + littlefs 1MB matching s3-app).
- `embedded-poc/m5stack-core2-app/src/main.rs` — 10 lines, calls
  `mfsk_app_shared::run::<BoardCore2>()`.

Hardware bring-up + retire (Phase 5):

- `git rm -r embedded-poc/m5stack-core2/` — old LX6 bench crate.
- Defer `embedded-poc/m5stack-s3/` (S3 bench) retirement — separate
  decision; not blocking #61.

Docs + workspace (Phase 6):

- Add `embedded-poc/m5stack-core2-app/CLAUDE.md`,
  `embedded-poc/mfsk-app-shared/CLAUDE.md`.
- Update `embedded-poc/CLAUDE.md` "Crates under embedded-poc" list.
- Update `docs/ROADMAP.md` Open follow-ups: mark #61 closed; remove
  the "weekend hardware bring-up" footnote on ε scheduling.

## Reused infrastructure (do not re-build)

- `embedded-shared::dual_core` — already board-agnostic. Both
  `BoardS3` and `BoardCore2` route through this for the
  pass2_split / stage3_split work. Commit `15913aa` already proved
  dual-core decode_block runs on Core2 LX6.
- `embedded-shared::wav_sim` — Q32 16/48 → 12 kHz resampler used by
  both bench and PDM-driven capture.
- `embedded-shared::stage1_inc` — incremental stage-1 spectrogram,
  no cfg-gating needed.
- `embedded-shared::internal_pool` — `.bss`-resident cs scratch,
  works on both DRAM sizes (Core2 ~280 KB usable vs S3 ~512 KB).
- `mfsk-core::ft8::decode_block` — the LX6 vs LX7 difference is the
  `fixed-point-bp` asm dot product (LX6-gated already) and the
  `fft-extern` planner (works on both via `esp-dsp`). No mfsk-core
  changes needed.
- `mipidsi` crate (already in s3-app deps) supports ILI9342C in
  addition to ST7789P3 — no new display crate dependency.

## Implementation phases

Each phase is one PR off the same tracking issue. Phases 1-2 are
mechanical refactors that ship before the weekend; phases 3-5 land
during the weekend hardware window.

### Phase 1 — Extract `mfsk-app-shared` library (1 day)

Pure code move with a `pub fn run` entry. The S3 build must remain
byte-identical (same final ELF size ± alignment).

1. Create `embedded-poc/mfsk-app-shared/` with `[lib]` Cargo.toml.
2. Move all 20 source files from `m5stack-s3-app/src/` (except
   `main.rs`) into `mfsk-app-shared/src/`.
3. Convert `mod` declarations: what was `mod adif;` in `main.rs`
   becomes `pub mod adif;` in `mfsk-app-shared/src/lib.rs`.
4. Expose `pub fn run() -> !` matching the current `main` body.
5. Shrink `m5stack-s3-app/src/main.rs` to a 10-line shim.
6. Update `m5stack-s3-app/Cargo.toml` — add
   `mfsk-app-shared = { path = "../mfsk-app-shared" }`, drop now-
   library-owned deps.

Acceptance: `cargo build --release` green on m5stack-s3-app, S3 app
boots and decodes identically to commit `27425cf` baseline.

### Phase 2 — Introduce `Board` trait + extract `BoardS3` (2 days)

Type-level seam so a second board impl plugs in cleanly.

1. Define `mfsk-app-shared/src/board.rs`:
   ```rust
   pub trait Board {
       // Pinout
       const LCD_WIDTH: u16;
       const LCD_HEIGHT: u16;
       const I2C_BUS: u8;
       // ...
       // Init seams
       fn init_pmic(i2c: &mut I2cDriver) -> Result<()>;
       fn init_lcd(spi: SpiHandle, pins: LcdPins) -> Result<LcdSurface>;
       fn init_audio_capture(...) -> Result<AudioCaptureHandle>;
       fn poll_buttons() -> ButtonEvents;
       // Identifier for log lines
       const NAME: &'static str;
   }
   ```
2. Move existing S3-specific init from `display.rs` / `pmic.rs` /
   `audio.rs` / `buttons.rs` / `board.rs` into
   `mfsk-app-shared/src/boards/s3.rs` as `BoardS3` impl.
3. Make `display.rs` / `audio.rs` / `pmic.rs` / `buttons.rs` generic
   over `B: Board` (or rename — module structure can stay).
4. `lib.rs::run<B: Board>()` plumbs the board through to call sites.
5. Update m5stack-s3-app shim to `run::<BoardS3>()`.

Acceptance: S3 build green, S3 boot + decode unchanged. Recall on
qso3_busy reference WAV must not regress. ELF size delta < ±2 KB.

### Phase 3 — Implement `BoardCore2` (3-4 days)

Port the missing LX6 peripheral glue. Drives the bulk of new code.

1. **AXP192 PMIC** (`boards/core2/pmic_axp192.rs`) — I2C 0x34:
   - LDO2 = LCD logic (3.3 V), LDO3 = LCD backlight (2.8 V),
     DC-DC1 = ESP main rail. Power-on sequence + button charge
     state register.
   - Reference: M5Unified `power_axp192.cpp` minimum subset.
2. **ILI9342C LCD** (`boards/core2/lcd.rs`) — SPI:
   - SCK = GPIO18, MOSI = GPIO23, MISO = GPIO38, CS = GPIO5,
     DC = GPIO15, RST = (AXP192-controlled).
   - Use `mipidsi::Builder::new(ILI9342CRgb565::default(), di)`.
3. **PDM mic** (`boards/core2/audio_pdm.rs`) — I2S0 PDM mode:
   - CLK = GPIO0, DIN = GPIO34, mono, 16 kHz, 16-bit.
   - Use `esp-idf-hal::i2s::I2sDriver` PDM RX config; downsample to
     12 kHz with the existing `wav_sim`-style Q32 linear resampler.
   - Reference: M5Unified `audio_class.cpp` PDM init; SPM1423
     datasheet.
4. **Buttons** (`boards/core2/buttons.rs`) — BtnA = GPIO39,
   BtnB = GPIO38, BtnC = GPIO37, all active-low. Mirror the s3
   poll shape.
5. **MPU6886 IMU** — out of scope for #61. Stub `Board::imu()` to
   `None` for Core2; revisit if/when the controller UI uses motion.

Acceptance: `cargo check --target=xtensa-esp32-espidf` green from a
stub `m5stack-core2-app` Cargo.toml. No hardware flash yet.

### Phase 4 — Create `m5stack-core2-app` binary crate (0.5 day)

Pure config wiring. The actual code is already in mfsk-app-shared.

1. `embedded-poc/m5stack-core2-app/Cargo.toml` — `[bin]` only, deps
   = mfsk-app-shared + log + esp-idf-svc (matched to s3-app
   version).
2. `.cargo/config.toml` — target `xtensa-esp32-espidf`, runner
   `espflash flash --monitor --before no-reset --after no-reset`,
   `MCU=esp32` env.
3. `sdkconfig.defaults` — QUAD PSRAM, BT off, WiFi optional,
   `SPIRAM_MALLOC_ALWAYSINTERNAL = 4096` (same dual-core threshold
   as S3), partition table custom.
4. `partitions.csv` — factory 3MB + littlefs 1MB layout matching
   s3-app.
5. `src/main.rs` — 10 lines:
   ```rust
   fn main() -> ! {
       esp_idf_svc::sys::link_patches();
       mfsk_app_shared::run::<mfsk_app_shared::boards::BoardCore2>()
   }
   ```
6. `build.rs` — same partition-CSV-path-from-`CARGO_MANIFEST_DIR`
   shim as commit `6b8ca96`.

Acceptance: `cargo build --release` green for Core2 target on host
(no flash yet).

### Phase 5 — Hardware bring-up + retire old crate (weekend window)

1. Source `~/export-esp.sh`, build, flash to real Core2 hardware
   via `embedded-poc/scripts/flash-monitor.sh`.
2. Boot, confirm: AXP192 power-up message → LCD shows boot banner →
   PDM mic captures audio → dual-core decode pipeline produces
   decodes for the same qso3_busy WAV input the bench used.
3. Bench parity: confirm Core2 dual-core hits its current
   `~3.22 s/slot` baseline (commit `15913aa`). Capture log to
   `embedded-poc/m5stack-core2-app/logs/core2_fold_in_2026-05-1X.log`.
4. S3 regression: re-flash s3-app + qso3_busy reference, confirm
   recall ≥ baseline (7 total per memory `project_release_0_6_x`).
5. `git rm -r embedded-poc/m5stack-core2/` — old bench crate
   retired.
6. Update workspace `Cargo.toml` member list.

### Phase 6 — Docs + workspace sync (0.5 day)

1. New `embedded-poc/m5stack-core2-app/CLAUDE.md` — crate-specific
   build/flash overrides (mirror s3-app/CLAUDE.md structure;
   USB-OTG note becomes "no USB-OTG on LX6 stock board").
2. New `embedded-poc/mfsk-app-shared/CLAUDE.md` — library crate
   notes (board trait, sub-module map).
3. Update `embedded-poc/CLAUDE.md` "Crates under embedded-poc"
   section.
4. Update `docs/ROADMAP.md` Open follow-ups: mark #61 closed;
   remove the "weekend hardware bring-up" footnote on ε scheduling
   (its prerequisite is now satisfied).
5. Memory note via `project_esp32_port.md` — record dual-core LX6
   under unified-app structure with new fold-in numbers.

## Out of scope for #61

- LX6-LX7 SIMD parity (asm dot product stays LX6-gated, per #61).
- LCD UI feature parity — Core2 UI may start as log-only panel.
  Full QSO UI port to ILI9342C is a follow-up.
- MPU6886 IMU support on Core2.
- WiFi-mode polish on Core2 — boots and connects, but UDP-log
  binding behaviour on QUAD PSRAM untested.
- Retiring `embedded-poc/m5stack-s3/` (S3 bench crate) — separate
  decision once we confirm s3-app covers all bench needs.

## Risks + mitigations

- **mipidsi ILI9342C parity**: the crate's ILI9342C driver may
  diverge from M5Stack's stock orientation / colour-order tuning.
  Plan B is a hand-written init sequence (~30 lines, port from
  M5Unified `Panel_M5Stack.cpp`).
- **PDM clock stability**: SPM1423 has been known to produce a
  ~50 ppm offset that breaks slot alignment. If it surfaces, use
  the same NTP slot-boundary anchor s3-app already uses
  (`time_sync.rs`) — already board-agnostic.
- **TLSF heap fragmentation on Core2**: known LX6 bug (see
  `embedded-poc/m5stack-core2/CLAUDE.md`). The s3-app D-pattern
  workaround already lives in shared code paths, so this risk is
  inherited not new.
- **QUAD vs Octal PSRAM throughput**: dual-core on LX6 QUAD
  measured ~3.22 s in `15913aa`. Acceptance is "no regression from
  that baseline" — not S3 parity.

## Verification (end-to-end)

After Phase 5 lands:

```sh
# S3 app — no regression (baseline = current main)
source ~/export-esp.sh
cd embedded-poc/m5stack-s3-app
cargo build --release
../scripts/flash-monitor.sh \
    target/xtensa-esp32s3-espidf/release/m5stack-s3-app \
    logs/s3_post_core2_fold_$(date +%Y-%m-%d).log \
    120
# Expect: boot OK, qso3_busy decode = 7 hits, no recall drop

# Core2 app — fold-in works
cd ../m5stack-core2-app
cargo build --release
../scripts/flash-monitor.sh \
    target/xtensa-esp32-espidf/release/m5stack-core2-app \
    logs/core2_fold_in_$(date +%Y-%m-%d).log \
    120
# Expect: AXP192 power-up, LCD banner, PDM capture, dual-core
#         decode pipeline ≤ 3.5 s/slot on qso3_busy

# Host parity (no behavioural changes from this PR)
cd ../../mfsk-core
cargo test                                  # default features
cargo test --features fixed-point          # embedded LLR parity
cargo test --test ft8_qso3_apoff_recall    # WSJT-X golden
```
