---
name: flash-capture
description: Build, flash and capture a serial log from an M5Stack board (CoreS3, S3, Core2) through flash-monitor.sh. Use when an embedded change needs verifying on hardware and the user agrees to flash.
disable-model-invocation: true
---

# Flash and capture a log

Read first: `embedded-poc/CLAUDE.md` and the board crate's `CLAUDE.md`. Procedure of record:
`docs/notes/EMBEDDED_WORKFLOW.md` ("Capturing logs from a flashed device"). The user owns the hardware:
confirm the board is connected (and, for the CoreS3, the IC-705 / USB host state) before flashing.

## Steps

```sh
source ~/export-esp.sh
cd embedded-poc/<crate>              # m5stack-cores3-app | m5stack-s3-app | m5stack-core2-app | m5stack-s3
cargo build --release --bin <bin>    # features: the shipped numeric path is `fixed-point`
../scripts/flash-monitor.sh \
    target/<triple>/release/<bin> \
    logs/<bin>_<tag>_$(date +%Y-%m-%d).log \
    180
```

`<triple>`: `xtensa-esp32s3-espidf` (S3, CoreS3) or `xtensa-esp32-espidf` (Core2). Run it in the background
and wait for it to exit; don't poll the log. A flash log is thousands of lines where only a few matter: reading
it through `runner`, which returns the lines asked for, is the case `docs/notes/AGENT_WORKFLOW.md` §3 names.
Never run `cargo fmt --all` with the `+esp` toolchain.

## Read the log for these before anything else

- `Segment ... has not changed, skipping write` and a ~5 s finish: **not a flash**. The board still runs the old
  binary. Change a source line (a `log::info!`) and rebuild.
- `boot: Failed to verify partition table` / `exceeds flash chip size`: the flash-size header; the script
  auto-detects it, so pass `FLASH_SIZE` explicitly only if detection failed (it warns).
- Board parked in DOWNLOAD mode (`rst:0x15 ... waiting for download`) after the window expires: a short press of
  the board button boots the app; nothing is lost.

Never start a second `espflash monitor` or read `/dev/ttyACM*` / `/dev/cu.usbmodem*` directly (the hook denies
both). Before touching `decode_block` or the production app crates, read the embedded performance log
(CLAUDE.md, Memory).
