# embedded-poc — agent notes (shared)

Cross-board workflow for the ESP32 Xtensa crates under
`embedded-poc/`. Per-crate `CLAUDE.md` files cover board-specific
overrides (target triple, port, PSRAM mode) and point at this file
for the shared setup / debug list. The repo-root `CLAUDE.md` covers
flash-and-capture conventions (`scripts/flash-monitor.sh`,
`logs/` layout) that apply to every embedded crate here.

**No continuous lint gate.** The repo-root `Cargo.toml`'s `exclude =
["embedded-poc", ...]` keeps these crates out of the host Cargo
workspace (they need the `+esp` toolchain, which the fast host CI
jobs don't have), so neither the pre-commit hook's `cargo clippy
--workspace ...` nor CI's lint-gate job (`ci.yml`'s `rustfmt +
clippy` job) ever touches this directory — `dead_code` and every
other lint here accumulates silently until someone manually runs
`cargo +esp build`/`clippy` per crate (2026-08-14 dead-code sweep
found exactly one: a stale `field is never read` in
`embedded-shared::wspr_dual_core::WorkerStack`, fixed). Adding a CI
job for this was considered and declined for now — same
cost/benefit tradeoff as the tier-C sensitivity sweeps not being in
CI (`+esp` toolchain setup, esp-idf checkout, is not cheap for a
lightweight runner). If you're doing any kind of cleanup/audit pass
across the repo, remember to check here too — nothing else will.

## One-time setup (already done on this machine)

- `~/.rustup/toolchains/esp/` — Xtensa-fork Rust toolchain installed
  via [`espup`](https://github.com/esp-rs/espup). Selected
  automatically per crate by `rust-toolchain.toml`
  (`channel = "esp"`). Covers both `xtensa-esp32-espidf` (LX6) and
  `xtensa-esp32s3-espidf` (LX7).
- `~/.espressif/` — esp-idf checkout / tools managed by `embuild`
  (downloaded on first build into `.embuild/` inside each crate).
- `~/export-esp.sh` — sets `PATH` and `LIBCLANG_PATH` for the
  Xtensa toolchain. **Must be sourced** before any cargo invocation,
  otherwise `bindgen` fails to find clang and the Xtensa GCC
  binutils aren't on PATH.
- `~/.cargo/bin/espflash` — flasher (espflash 4.x).

## Build + flash + monitor (shared template)

The per-crate `CLAUDE.md` files override the target triple and bin
name; the three-step workflow itself is identical:

```sh
# 1. Source the cross-dev env (PATH + LIBCLANG_PATH).
source ~/export-esp.sh

# 2. Build for the crate's target triple.
cd embedded-poc/<crate>
cargo build --release

# 3. Flash and capture, via the wrapper that gets the espflash
#    reset flags right (avoids dropping S3 USB-OTG boards into
#    DOWNLOAD mode on the second flash).
../scripts/flash-monitor.sh \
    target/<triple>/release/<bin> \
    logs/<bin>_<tag>_$(date +%Y-%m-%d).log \
    90    # capture seconds (optional, default 90)
```

`cargo run --release` works the same when the runner in
`.cargo/config.toml` is set up and the shell is interactive
(espflash auto-detects a single port). The explicit 3-step form
is what the user types under tee / piped redirection.

## LX6 (Core2) vs LX7 (S3) — comparison table

| | M5Stack Core2 (LX6) | M5StickS3 / S3-app (LX7) | M5Stack CoreS3 / CoreS3-app (LX7) |
|---|---|---|---|
| Target triple | `xtensa-esp32-espidf` | `xtensa-esp32s3-espidf` | `xtensa-esp32s3-espidf` |
| Bench bin | (retired `#61` Phase 3 — `m5stack-core2-app` runs the wav_sim decode loop instead) | `mfsk-core-m5stack-s3` | (none; reuses s3 bench if needed) |
| Production app | `m5stack-core2-app` (FT8 controller) | `m5stack-s3-app` (FT8 controller, **demo / Phase 1.5 acoustic**) | `m5stack-cores3-app` (FT8 controller, **main UAC target**, planned) |
| PMIC | AXP192 (0x34) | M5PM1 (0x6E) | AXP2101 + AW9523B I/O expander (0x58); **AW9523B P1 = BUS_OUT_EN for USB host** |
| Flash / PSRAM | 16 MB / 8 MB (default) | 8 MB / 8 MB Octal (`CONFIG_SPIRAM_MODE_OCT=y`, ~80 MB/s); Quad on M5Stamp S3 | 16 MB / 8 MB **Quad** (`CONFIG_SPIRAM_MODE_QUAD=y`) |
| Internal DRAM | ~280 KB usable | ~512 KB | ~512 KB |
| USB host capable | No (no USB peripheral; flashes via CP210x UART) | **No** (silicon yes, board no — no VBUS source, ID pin unwired, see memory `project_m5stick_s3_no_usb_host`) | **Yes** — AXP2101 + AW9523B BUS_OUT_EN drives VBUS boost |
| Port enumeration | `/dev/ttyACM0` (CP2104) | `/dev/ttyACM0` (USB-Serial-JTAG, native S3) | `/dev/ttyACM0` (USB-Serial-JTAG via CH9102 bridge on CoreS3) |
| SIMD | None | esp-dsp `_ae32_` asm (LX6/LX7 shared, scalar single-issue) — LX7 PIE `_aes3_` migration pending, see `docs/notes/PHASE_D_PIE_SIMD.md` | esp-dsp `_ae32_` asm (same Phase D D1 migration applies) |
| LCD | ILI9342C 320×240 landscape | ST7789P3 135×240 portrait | ILI9342C 320×240 landscape (same as Core2) + FT6336U capacitive touch |
| Audio codec | none | ES8311 (mono mic + speaker amp) | ES7210 (dual mic) + AW88298 speaker amp |
| Buttons / input | none (touch deferred Phase 2.5) | KEY1 / KEY2 GPIO 11/12 | none (touch FT6336U via I2C, Phase 6-Core) |
| `release.opt-level` | `1` | `1` (shared Xtensa-Rust LLVM regression on `s`/`z` with f32-select patterns — `core::pipeline` ships an arithmetic-form workaround) | `1` (same) |
| `SPIRAM_MALLOC_ALWAYSINTERNAL` | n/a | `4096` (mandatory for dual-core; `16384` drains internal DRAM with cs Box × 2 workers → tlsf corruption) | `4096` (same dual-core constraint) |

## Trouble we've already debugged (cross-board)

- **`espflash::no_serial`** — device not connected, or
  `/dev/tty*` permission denied (user not in `dialout`).
- **`espflash::dialoguer_error: not a terminal`** — espflash
  auto-detected multiple ports and tried to prompt; pass `--port`
  explicitly, or use `scripts/flash-monitor.sh` which forwards
  `--port` automatically.
- **bindgen / `unable to find libclang`** — forgot to source
  `~/export-esp.sh`. `LIBCLANG_PATH` must point at the bundled
  esp-clang (not system clang).
- **wrong toolchain (`error: toolchain 'esp' is not installed`)**
  — reinstall via `espup install`. Don't try to use stable Rust here.
- **`tlsf_malloc` heap corruption mid-sweep** — known bug when
  `decode_block` is called directly from a long-running bench
  loop; allocator state gets corrupted partway through. Production
  bench `main.rs` works around it by inlining the per-iteration
  steps (the "D pattern") instead of calling `decode_block` as a
  single function.
- **"Segment … has not changed, skipping write"** — re-flashing
  the same ELF finishes in ~5 s but the chip still runs the
  previous binary. Touch a source file (e.g. bump a `log::info!`
  line) to force a real rewrite; expect ~15-25 s for a real
  factory-partition write. See repo-root CLAUDE.md for full
  context on this and the `--before no-reset` flag that
  `scripts/flash-monitor.sh` passes.

## Crates under `embedded-poc/`

- **`m5stack-s3/`** — S3 LX7 compute bench. Decoder-only,
  WAV-fed `rx_wavsim` is the primary driver. (The Core2 sibling
  bench was retired in `#61` Phase 3 — `m5stack-core2-app` covers
  the equivalent wav_sim path in a production-app shape.)
- **`m5stack-s3-app/`** — M5StickS3 FT8 controller (LCD UI + QSO
  FSM + WiFi UDP log streaming + ES8311 audio). **Repositioned
  2026-05-17 as demo / acoustic-fallback path** after Phase 1 UAC
  hardware verification confirmed M5StickS3 board cannot do USB
  host (see memory `project_m5stick_s3_no_usb_host`). UAC code
  (`uac.rs` ~445 lines) is board-agnostic and stays as canonical
  reference for the CoreS3 lift in Phase 0-Core. Phase 1.5 (planned)
  adds internal MEMS-mic acoustic capture as the Stick demo path.
- **`m5stack-cores3-app/`** — M5Stack CoreS3 FT8 controller (LX7,
  **main production target**, planned 2026-05-17 onward). Same
  `mfsk-app-shared` consumer pattern as core2-app. Distinguishing
  HW: AXP2101 PMIC + AW9523B I/O expander (P1 = BUS_OUT_EN drives
  VBUS boost — must be HIGH before `usb_host_install()`), ILI9342C
  LCD with FT6336U capacitive touch, ES7210 dual-mic codec. See
  Phase B-Core in `docs/notes/ROADMAP.md` for the work breakdown.
  Also hosts `src/bin/wspr_bench.rs`, a **separate** binary for the
  WSPR embedded RX work (Phase E in `docs/notes/ROADMAP.md`, issue
  #260) — decoder-level, not part of the FT8 controller line above;
  build-time switches `MFSK_WSPR_SPOT`/`MFSK_WSPR_BENCH_WIFI`. See
  `docs/reference/EMBEDDED.md`'s "WSPR on embedded" section. A third
  bin, `src/bin/wspr_app.rs` (`wspr-app`), is the actual receiver UI
  built on top of that decode work — stations display + spot history
  on the CoreS3's own 320×240 panel, settings edited from a browser
  (`mfsk-app-shared`'s `http_config`/`ntp`/`settings` modules) rather
  than any on-device input (CoreS3 has none). Live audio capture is
  still out of scope (blocked on #163, same as `wspr-bench`) — every
  slot decodes the same baked golden baseband. See memory
  `project_wspr_app_cores3_ui` for the full design/status. A fourth
  bin, `src/bin/fst4_bench.rs` (`fst4-bench`), is issue #306's
  decoder-only FST4-60 bench — a separate feasibility question from
  the WSPR track above (FST4's LDPC/BP/OSD path shares nothing
  algorithmically with WSPR's Fano decoder), sharing only the
  baked-golden-asset pattern. `include_bytes!`s a host-baked FFT cache
  (`fst4::decode`'s `FST4_60A_DOWNSAMPLE.fft1_size = 746_496`-point
  transform, past the ESP-DSP FFT ceiling same as WSPR's own wideband
  stage) generated by the `#[ignore]`d `fst4_bake_golden_precomputed`
  in `mfsk-core/tests/fst4_wsjtx_samples.rs`. This bin's baked assets
  (~7.16 MiB) are why `partitions.csv`'s `factory` partition grew from
  3 MiB to 9 MiB 2026-08-16 — shared across every `[[bin]]` in this
  crate, so re-flashing `wspr-app` after that change resets its
  littlefs-stored settings (re-enter via the HTTP config server).
- **`m5stack-core2-app/`** — Core2 (LX6) sibling of the above
  (`#61` Phase 2). Same `mfsk-app-shared` consumer, board-specific
  HW drivers swapped: AXP192 PMIC, ILI9342C LCD (via mipidsi's
  ILI9341 model in landscape), no buttons (Core2 touch deferred),
  no audio (decode pipeline fed by `wav_sim`). See its `CLAUDE.md`
  for the Core2-specific bring-up sequence and SPI baud caveat.
- **`mfsk-app-shared/`** — board-agnostic app logic (Issue #61
  Phase 1). QSO FSM, time sync, TX picker, SNR norm, NVS boot
  mode, log fanout (`LogFanout` / `FanoutLogger`), WiFi STA +
  UDP datagram sink, UI data structures + `embedded-graphics`
  draw routines, ADIF + flash log placeholders. Shared/board
  boundary is **data flow** (channels + shared state mutex), not
  callbacks — no traits cross the boundary. Consumed by both
  `m5stack-s3-app` and `m5stack-core2-app`.
- **`embedded-shared/`** — `no_std` decode-pipeline crate shared
  between the bench / app crates. Streaming pipeline, dual-core
  dispatch, resamplers, BASIS scratch helpers. Distinct from
  `mfsk-app-shared`: this is the decoder-level shared layer
  (FFT planner / dual_core / stage1_inc), while
  `mfsk-app-shared` is the controller-level shared layer (QSO /
  UI / WiFi). Both are board-agnostic. Also has `wspr_dual_core.rs`
  (persistent dual-core worker for the WSPR candidate loop, Phase E)
  — an FT8-independent sibling of `dual_core.rs`, not a shared
  abstraction between the two protocols.
- **`idf-component/`** — esp-idf component shim that wraps
  `mfsk-ffi-ft8` so C-only ESP-IDF projects can pull the FT8
  decoder in without writing Rust glue.
- **`scripts/`** — `flash-monitor.sh`, `udp-log-listen.sh`.
  Both actively used; see repo-root CLAUDE.md for why.
- **`assets/`** — vendored WSJT-X reference WAVs used by host
  integration tests (`asset_path!` macro) and by the embedded
  WAV-feed bench.
