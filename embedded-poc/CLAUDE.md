# embedded-poc — agent notes (shared)

Cross-board workflow for the ESP32 Xtensa crates under
`embedded-poc/`. Per-crate `CLAUDE.md` files cover board-specific
overrides (target triple, port, PSRAM mode) and point at this file
for the shared setup / debug list. The repo-root `CLAUDE.md` covers
flash-and-capture conventions (`scripts/flash-monitor.sh`,
`logs/` layout) that apply to every embedded crate here.

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

| | M5Stack Core2 (LX6) | M5StickS3 / S3-app (LX7) |
|---|---|---|
| Target triple | `xtensa-esp32-espidf` | `xtensa-esp32s3-espidf` |
| Bench bin | (retired `#61` Phase 3 — `m5stack-core2-app` runs the wav_sim decode loop instead) | `mfsk-core-m5stack-s3` |
| Production app | `m5stack-core2-app` (FT8 controller) | `m5stack-s3-app` (FT8 controller) |
| PSRAM mode | (default) | Octal (`CONFIG_SPIRAM_MODE_OCT=y`, ~80 MB/s); Quad on M5Stamp S3 — set `_MODE_QUAD=y` |
| Internal DRAM | ~280 KB usable | ~512 KB |
| Port enumeration | `/dev/ttyACM0` | `/dev/ttyACM0` (USB-Serial-JTAG, native S3); `/dev/ttyUSB0` (CP210x bridge on some boards) |
| SIMD | None | PIE (auto-picked by `esp-dsp` at build); no hand-written intrinsics |
| `release.opt-level` | `1` | `1` (shared Xtensa-Rust LLVM regression on `s`/`z` with f32-select patterns — `core::pipeline` ships an arithmetic-form workaround) |
| `SPIRAM_MALLOC_ALWAYSINTERNAL` | n/a | `4096` (mandatory for dual-core; `16384` drains internal DRAM with cs Box × 2 workers → tlsf corruption) |

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
- **`m5stack-s3-app/`** — production FT8 controller (LCD UI +
  QSO FSM + WiFi UDP log streaming + ES8311 audio + planned UAC).
  Bin crate: HW drivers (`audio` / `display` / `pmic` / `buttons` /
  `board`) + `main.rs` orchestration + `decode_pipeline.rs` (still
  here in Phase 1 since it owns the heap_caps BASIS alloc; carved
  out only if Phase 2 reveals a clean cut).
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
  UI / WiFi). Both are board-agnostic.
- **`idf-component/`** — esp-idf component shim that wraps
  `mfsk-ffi-ft8` so C-only ESP-IDF projects can pull the FT8
  decoder in without writing Rust glue.
- **`scripts/`** — `flash-monitor.sh`, `udp-log-listen.sh`.
  Both actively used; see repo-root CLAUDE.md for why.
- **`assets/`** — vendored WSJT-X reference WAVs used by host
  integration tests (`asset_path!` macro) and by the embedded
  WAV-feed bench.
