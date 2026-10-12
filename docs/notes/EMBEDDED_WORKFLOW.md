# Embedded targets: build, flash, capture logs

Moved out of the repo-root `CLAUDE.md` (2026-10-12). Board-specific notes live in `embedded-poc/CLAUDE.md` and each crate's own `CLAUDE.md`.

## Embedded targets

The active production crates are `embedded-poc/m5stack-s3-app/` (S3
LX7, repositioned as **demo / acoustic-fallback** in the 2026-05-17
pivot — the StickS3 board can't source VBUS for USB host),
`embedded-poc/m5stack-core2-app/` (Core2 LX6, wav_sim only — no USB
peripheral on classic ESP32), and `embedded-poc/m5stack-cores3-app/`
(S3 LX7, **main UAC controller target** — CoreS3 has AXP2101 +
AW9523B for proper USB-OTG host mode). Phase 0-Core (board bring-up)
and Phase 1-Core (UAC host code) both shipped 2026-05-23, and
Phase 1-Verify — live IC-705 hardware confirmation, issue #163 —
**cleared 2026-08-23**: ten unbroken minutes of UAC capture, 125 MB,
zero errors, with WiFi associated throughout (the log is kept at
`embedded-poc/m5stack-cores3-app/logs/uac_stream_2026-08-23.log`).
Getting there took three layers off the enumeration path and then two
stack overflows that presented as heap corruption — read
`embedded-poc/CLAUDE.md`'s "USB host VBUS on CoreS3" and "Stacks,
heaps, and the space between them" before touching that board.

**It decodes off the air now (2026-08-23/24)**: FT8 at six to eight
stations per slot against an IC-705 on 40 m, WSPR at
`slot 1 src=uac decoded 1 station(s)`. The board carries all four
receivers in one image, chosen from the touch panel. Operating notes
are in `docs/reference/MANUAL_M5STACK_CORES3.md` (and `.ja.md`); the
crate has its own `CLAUDE.md` for what breaks a session.
See `docs/notes/ROADMAP.md` Phase B-Core for what #163 unblocks. `m5stack-s3-app` and
`m5stack-core2-app` each have their own `CLAUDE.md` covering
board-specific bring-up, and `m5stack-cores3-app` now does too —
alongside the operator manual at
`docs/reference/MANUAL_M5STACK_CORES3.md` / `.ja.md`. This
section captures the shared workflow that's easy to forget between
sessions.

Shared code for these boards lives in `embedded-poc/embedded-shared/`
(decode-side glue, esp-dsp FFT/dotprod backends) and
`embedded-poc/mfsk-app-shared/` (UI/app logic, whose host-testable half
runs in the workspace as `hosttest/mfsk-app-shared`).
`embedded-poc/idf-component/` is the esp-idf bridge template. Note it
requires a Rust staticlib shim regardless — pure C cannot define the
`extern "Rust"` FFT-planner symbol — which is why `mfsk-ffi-ft8` was
retired: once a consumer is writing Rust, calling `mfsk_core` directly
is strictly simpler, and that is what all three boards here do.

**WSPR embedded RX (Phase E, issue #260) is a separate track from the
FT8-controller line above** — it never goes through `decode_block` or
the UAC/controller stack, so it isn't blocked on #163 the way Phase
B-Core is (both shared #163 only for live audio capture; everything
else already works against WAV-fed/synthetic baseband, and #163 itself
cleared 2026-08-23 on the FT8 controller). Lives in the
same `m5stack-cores3-app` crate as two separate binaries:
`src/bin/wspr_bench.rs` (timing measurement) and `src/bin/wspr_app.rs`
(the standalone receiver — the shared FT8 panel, WiFi, HTTP config, NTP, and
real UAC audio through `AudioSink`). Note that `wspr_app` *does* now
share `uac.rs` with the FT8 line; that path is proven on the FT8
controller but `wspr_app` itself has not been run against a radio yet
— open items in #313. See `docs/reference/EMBEDDED.md`'s "WSPR on
embedded" section and `docs/notes/ROADMAP.md` Phase E for status.

- **Build & flash via `espflash`**, not host cargo. Each crate's
  `.cargo/config.toml` sets `runner = "espflash flash --monitor"`, so the
  basic user workflow is:
  ```sh
  cd embedded-poc/m5stack-cores3-app   # or m5stack-s3-app, m5stack-core2-app
  cargo run --release                  # builds + flashes + opens serial monitor
  ```
  In practice, for capturing per-session logs to a file, use
  `embedded-poc/scripts/flash-monitor.sh` (see next section) instead of
  the bare runner.
- The `+esp` Rust toolchain (Xtensa fork, espup-installed) is selected
  by each crate's `rust-toolchain.toml`. Target triple per
  `.cargo/config.toml`: `xtensa-esp32-espidf` for Core2 (LX6),
  `xtensa-esp32s3-espidf` for S3 / CoreS3 (LX7).
- `cargo check` from inside each crate validates code changes without
  flashing (~30-50 s with prebuilt esp-idf).
- Host CI never builds these crates (`ci.yml`'s trigger excludes
  `embedded-poc/**` apart from the test assets and the
  `mfsk-app-shared` sources `hosttest/` compiles). A green CI badge on an embedded-only change means
  the change was not compiled by anything.
- Logs from the device land in `embedded-poc/<crate>/logs/` — user has
  been capturing per-session sweep output there.
- The user actively flashes these boards during embedded work; do NOT
  assume "host check is enough" when changing `src/main.rs` or anything
  that affects the runtime path. Offer to flash and capture a new log.
- The compute-bench crate `embedded-poc/m5stack-s3/` still exists for
  S3-only decoder timing sweeps. The Core2 bench (`m5stack-core2/`) was
  retired in `#61` Phase 3 — `m5stack-core2-app` covers the same
  wav_sim decode path in a production-app shape.

## Capturing logs from a flashed device (ESP32 / S3)

Use `embedded-poc/scripts/flash-monitor.sh` — **never** roll your own
`espflash flash --monitor` + redirect, and never `cat /dev/ttyACM0`. Two
foot-guns this script avoids:

1. A **separate** `espflash monitor` invocation resets on connect, and
   on S3 USB-OTG boards that drops the chip into DOWNLOAD mode
   (`rst:0x15 USB_UART_CHIP_RESET … waiting for download`) so the
   just-flashed app never runs. The script avoids it by never starting
   a second process: one `espflash flash --monitor` covers both.

   It does **not** pass `--before no-reset --after no-reset`, as this
   note used to claim — those would break flashing outright.
   `espflash flash`'s own defaults are already what is wanted:
   `--before default-reset` is how it gets into the bootloader to write
   at all, and `--after hard-reset` is what starts the new image.

   Boards have still been found parked in DOWNLOAD mode twice
   (2026-08-23), both times *after* the capture window expired and
   `timeout` killed the pty rather than after the write itself. The
   port closing is the suspect, not the flash. Unconfirmed. A short
   press of the board's button boots the app; nothing is lost, since
   the image is already in flash and NVS is a different partition.

2. Re-flashing the same ELF prints "Segment … has not changed, skipping
   write" and finishes in ~5 s. **That is not a successful flash** — the
   chip still runs the previous binary. Touch a source file or change a
   `log::info!` line to force a real rewrite, and expect ~15-25 s for a
   real factory-partition write.

```sh
source ~/export-esp.sh
cd embedded-poc/m5stack-s3-app   # or m5stack-s3, m5stack-core2-app, m5stack-cores3-app
cargo build --release --bin <bin>
../scripts/flash-monitor.sh \
    target/<triple>/release/<bin> \
    logs/<bin>_<tag>_$(date +%Y-%m-%d).log \
    90    # capture seconds (optional, default 90; use ≥120 for fresh Core2 flashes — 1.3 MB binary takes ~55 s to write)
```


