# mfsk-core — agent notes

Repo-level notes for assistants working in this tree. Anything specific to
a sub-crate lives in its own `CLAUDE.md` or `README.md`; this file is for
cross-crate workflow that's easy to forget between sessions.

## Embedded targets

### `embedded-poc/m5stack-core2`

Real hardware: M5Stack Core2 (ESP32-D0WD-V3, Xtensa LX6, 8 MB PSRAM).

- **Build & flash via `espflash`**, not host cargo. The `.cargo/config.toml`
  in that crate sets `runner = "espflash flash --monitor"`, so the user's
  workflow is:
  ```sh
  cd embedded-poc/m5stack-core2
  cargo run --release          # builds + flashes + opens serial monitor
  ```
- The `+esp` Rust toolchain (Xtensa fork, espup-installed) is selected by
  `rust-toolchain.toml`. Host (`cargo check`/`build`) uses `xtensa-esp32-espidf`
  per `.cargo/config.toml`.
- `cargo check` from inside the crate is enough to validate code changes
  without flashing (~50 s with prebuilt esp-idf).
- Logs from the device land in `embedded-poc/m5stack-core2/logs/` —
  user has been capturing per-config sweep output there.
- The user has been actively flashing this throughout the
  decode_block-embedded work; do NOT assume "host check is enough" when
  changing main.rs or anything that affects the bench output. Offer to
  let the user flash and capture the new log.

## Capturing logs from a flashed device (ESP32 / S3)

Use `embedded-poc/scripts/flash-monitor.sh` — **never** roll your own
`espflash flash --monitor` + redirect, and never `cat /dev/ttyACM0`. Two
foot-guns this script avoids:

1. `espflash monitor` defaults to `--before default-reset`, which pulses
   DTR/RTS and on S3 USB-OTG boards drops the chip into DOWNLOAD mode
   (`rst:0x15 USB_UART_CHIP_RESET … waiting for download`). The script
   passes `--before no-reset --after no-reset` so the just-flashed app
   keeps running.
2. Re-flashing the same ELF prints "Segment … has not changed, skipping
   write" and finishes in ~5 s. **That is not a successful flash** — the
   chip still runs the previous binary. Touch a source file or change a
   `log::info!` line to force a real rewrite, and expect ~15-25 s for a
   real factory-partition write.

```sh
source ~/export-esp.sh
cd embedded-poc/m5stack-s3   # or m5stack-core2
cargo build --release --bin <bin>
../scripts/flash-monitor.sh \
    target/xtensa-esp32s3-espidf/release/<bin> \
    logs/<bin>_<tag>_$(date +%Y-%m-%d).log \
    90    # capture seconds (optional, default 90)
```

## Test fixture paths

Never hardcode absolute paths like `/home/ubuntu/...` or `/Users/...`
for test inputs. AI assistants tend to "fix" path failures by
translating to whichever local environment they happen to run in
(commit `119657a` flipped `/home/minoru/` → `/home/ubuntu/`), which
just relocates the bug.

- **In-repo assets**: use the `asset_path!` macro from
  `mfsk-core/tests/common/mod.rs` (integration tests) or
  `concat!(env!("CARGO_MANIFEST_DIR"), "/../embedded-poc/assets/<f>")`
  (unit tests under `src/`). Vendor the file under
  `embedded-poc/assets/` if it's not already there — the FT8 / JT9
  reference recordings already live there.
- **Out-of-tree user-machine assets** (e.g. the full WSJT-X tarball):
  `option_env!("WSJTX_SAMPLES_DIR")` and skip cleanly when unset.
- **Diagnostic output paths** (test writes a WAV for human inspection):
  `/tmp/...` literals are fine — the human-in-the-loop step assumes a
  known location. Don't replace these with `tempfile`.

## Memory

- `~/.claude/projects/-home-minoru-src-mfsk-core/memory/` holds the
  per-conversation auto-memory. `project_decode_block_embedded.md` is the
  authoritative log of the embedded-port performance journey — read it
  before touching `decode_block` or m5stack-core2/main.rs.
