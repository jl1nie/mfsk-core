# M5StickS3 FT8 controller — user manual

A handheld FT8 decoder + QSO controller built on M5StickS3 hardware,
running the `mfsk-core` Rust decoder on Xtensa LX7. Couples to an
IC-705 (or any radio with a compatible BLE CI-V or audio cable) for
slot-driven decode + automatic QSO sequencing.

This manual covers daily use: build, flash, configure, and operate.
For internal architecture (FFT pipeline, Goertzel kernel, sync
algorithms), see `mfsk-core/` source.

---

## 1. What it does

The same firmware boots into one of **seven modes** (selected via the
NVS-persistent `BootMode` — see §5). The two production modes are:

- **Acoustic** — the on-board ES8311 microphone captures the radio's
  audio acoustically (point the Stick at the IC-705 speaker grille).
  Decoder runs continuously, decoded callsigns appear on the LCD,
  IC-705 PTT is keyed via BLE CI-V when the operator picks a station.
  No cable to the radio required.
- **Qso** — same as Acoustic but with bidirectional I2S so the Stick's
  speaker outputs synthesised TX audio (instead of only PTT-keying
  the IC-705's internal mic source). Used when an audio cable
  connects the Stick's TRRS jack to the radio's PHONE/SP IN.

Five development / diagnostic modes round out the set:

- **Decode** — `wav_sim` plays a baked `qso3_busy.wav` on loop, used
  to confirm the decoder works without any audio source attached.
- **Wifi** — boots WiFi STA + UDP log sink, no decode. Used to verify
  AP credentials in `cfg.toml`.
- **CivTest** — exercises the BLE CI-V link to an IC-705 (read freq,
  toggle PTT, read mode). For pairing diagnostics.
- **TxTest** — synthesises a 1500 Hz FT8 tone on I2S TX for verifying
  the speaker / audio cable path without needing a real signal.
- **Uac** — USB-Host audio class. **Cannot run on M5StickS3 hardware**
  (board lacks VBUS source + ID pin); see `docs/EMBEDDED.md` for the
  CoreS3 successor path. The mode is present so the codebase boots
  for shared verification work.

---

## 2. Hardware

### Required

| Item | Purpose |
|---|---|
| **M5StickS3** | Host board (ESP32-S3 LX7, 8 MB Octal PSRAM, ES8311 codec, ST7789P3 135×240 LCD, KEY1 / KEY2) |
| **USB-C cable** | Power + serial console + firmware flash |
| **Transceiver with audio** | IC-705 strongly recommended (BLE CI-V supported out of the box) |

### Optional

| Item | Purpose |
|---|---|
| **3.5 mm TRRS cable** | Stick TRRS jack ↔ radio PHONE/SP IN. **Required for Qso mode TX**. Not needed for Acoustic mode. |
| **WiFi AP** | Streams `log::info!` over UDP to your PC (much easier to follow than scrolling the LCD). |
| **PC running `udp-log-listen.sh`** | Receives the UDP log stream. |

### Pinout reference

See `embedded-poc/m5stack-s3-app/src/board.rs` for the M5StickS3 GPIO map (KEY1 = GPIO 11, KEY2 =
GPIO 12, ES8311 I2C on GPIO 38/39, LCD SPI on GPIO 13/14/15/17/18,
etc.). The M5Stack official pinout PDF agrees byte-for-byte.

---

## 3. First-time setup

### 3.1. Toolchain

The firmware is Rust-on-Xtensa-LX7. You need the `esp` rust toolchain
+ ESP-IDF v5.x (managed automatically by `embuild`).

```sh
# One-time: install the espup-managed Xtensa toolchain.
curl -L https://github.com/esp-rs/espup/releases/latest/download/espup-x86_64-unknown-linux-gnu -o espup
chmod +x espup
./espup install
source ~/export-esp.sh

# Verify the `esp` toolchain is selected automatically when you cd
# into an embedded crate (rust-toolchain.toml drives this).
cd embedded-poc/m5stack-s3-app
rustup show active-toolchain   # → "esp"
```

`~/export-esp.sh` exports `PATH` + `LIBCLANG_PATH` for the Xtensa
toolchain. **Source it in every fresh shell** before `cargo`
invocations — otherwise `bindgen` cannot find `libclang` and the build
fails with confusing errors.

### 3.2. Build

```sh
cd embedded-poc/m5stack-s3-app
source ~/export-esp.sh        # see §3.1
cargo build --release
```

First build takes ~10 min (ESP-IDF + esp-dsp + esp-nimble compile).
Subsequent builds are ~30 s.

### 3.3. Flash + monitor

**Always use the wrapper script**, not `espflash flash --monitor`
directly:

```sh
../scripts/flash-monitor.sh \
    target/xtensa-esp32s3-espidf/release/mfsk-core-m5stack-s3-app \
    logs/first-boot-$(date +%Y-%m-%d).log \
    90    # capture seconds (≥ 60 for the bringup sequence)
```

Why the wrapper:
- `espflash monitor` defaults to `--before default-reset`, which on
  S3 USB-OTG boards drops the chip into DOWNLOAD mode after flash
  ("rst:0x15 USB_UART_CHIP_RESET … waiting for download"). The
  wrapper passes `--before no-reset --after no-reset`.
- Re-flashing an unchanged ELF prints "Segment … has not changed,
  skipping write" and **does not actually re-flash** — the chip keeps
  running the old binary. The wrapper logs this clearly so you don't
  chase phantom behaviour. If you see this, touch a source file (bump
  a `log::info!` line) and rebuild.

A real flash takes 15-25 s for the S3's factory partition. The
monitor will then capture stdout from the chip for `90` seconds (or
whatever number you pass), writing to the log path.

### 3.4. `cfg.toml`

Copy the gitignored template and fill in real values:

```sh
cp cfg-sample.toml cfg.toml
# Edit cfg.toml — see §4.
```

`cfg.toml` is read by `build.rs` at compile time and the values are
injected via `cargo:rustc-env=`, so **any change requires a rebuild**.
This is intentional: secrets never make it into git, and the rebuild
forces the operator to acknowledge a credential change.

---

## 4. `cfg.toml` reference

```toml
[wifi]
ssid = "your-ssid-here"        # empty → WiFi init skipped
psk  = "your-passphrase-here"
pc_ip = "auto"                  # "auto" / "255.255.255.255" / "192.168.x.y"
port  = 9999                    # UDP log target port

[station]
call = "JL1NIE"                 # operator callsign — empty → QSO FSM idle
grid = "PM95"                   # 4-character Maidenhead grid
```

### `[wifi]`

- **`ssid` / `psk`**: home WiFi credentials. Leave `ssid = ""` (or
  drop the whole `[wifi]` section) to skip WiFi init — the firmware
  still boots, just without UDP log. Useful for portable use.
- **`pc_ip`**:
  - `"auto"` (default): firmware computes the directed subnet
    broadcast from the DHCP-assigned IP (e.g. `192.168.1.42/24` →
    `192.168.1.255`). Most reliable: APs commonly drop limited
    broadcast (`255.255.255.255`) but pass directed broadcast.
  - `"255.255.255.255"`: limited broadcast. Works on simple LANs.
  - `"192.168.x.y"`: unicast to a fixed PC IP. Most reliable when the
    PC IP is stable.
- **`port`**: UDP datagram port (default 9999). On the PC side:
  `embedded-poc/scripts/udp-log-listen.sh 9999`.

### `[station]`

- **`call`**: operator callsign. Empty → QSO FSM stays in Idle and the
  auto-CQ menu toggle is a no-op. Filled → CQ messages address-block
  uses this call.
- **`grid`**: 4-character Maidenhead grid for CQ messages.
- **On-device editing is intentionally not supported** — re-edit +
  rebuild is the canonical workflow. Two-button entry of a full
  callsign is error-prone, and a mis-typed callsign causes wrong-call
  QRM.

### WSL2 PC listener

For UDP log on WSL2 (Linux on Windows): you need a Hyper-V firewall
rule to accept inbound UDP 9999 (the mirrored-mode default blocks
it). Search `git log --grep "Phase 0.6"` for the
`New-NetFirewallHyperVRule` recipe.

---

## 5. `BootMode` — selecting and switching modes

The chosen `BootMode` is stored in NVS (`mfsk` namespace, key
`boot_mode`) and persists across power cycles. It is **not** a
build-time choice — the same flashed firmware can run any mode.

### Mode cycle

```
Decode → Wifi → Acoustic → CivTest → TxTest → Qso → Uac → Decode → …
```

### Two ways to change mode

| When | How | Effect |
|---|---|---|
| **At boot** | Hold **KEY1** (front button) while plugging in / pressing the reset side button | Cycle to the NEXT mode (one step in the order above), persist to NVS, then boot into it |
| **While running** | **Long-press KEY2** (top button) for ≥ 1 s | Same: cycle to next mode, persist, `esp_restart()` |

The currently active mode is shown:
- In the boot serial log (`boot_mode: QSO`, `boot_mode: ACOUSTIC`, …)
- On the LCD status bar (top row, right side)

### Recommended modes for daily use

- **Acoustic**: most operators. Just point the Stick at the radio
  speaker. BLE CI-V optional but recommended (auto PTT on QSO).
- **Qso**: when you have a TRRS cable and want the Stick to actually
  transmit (instead of just keying the radio's internal mic).

The other modes are diagnostic — most users will not need them
day-to-day.

---

## 6. Decode pipeline architecture (post Phase 1.7.7)

Audio in → decoded callsigns out, ~1.5–2 s after each FT8 slot ends.

```
ES8311 mic / TRRS in   ← 48 kHz stereo I2S RX (one channel used)
        │
        │  audio::capture_tx_thread  (LX7 PRO_CPU, prio 5)
        │  ─ 48k → 12k Q32 resampler (per-channel)
        │  ─ slot-end signal when 12_000 × 15 = 180_000 samples captured
        ▼
chunk_q  ← ChunkMsg::{Samples(Vec<i16>), SlotEnd { wav_idx, total_samples }}
        │
        │  stage1_inc::worker  (APP_CPU, prio 6 — preempts dsp_worker)
        │  ─ accumulate audio, lock auto-gain shift after 1 s
        │  ─ per-pair 3840-pt sc16 FFT every NSPS/2 = 960 samples
        │  ─ rectangular window (tone_step_bins = 2.0 exact at NFFT=3840)
        │  ─ two-for-one real-FFT trick (1 complex FFT = 2 frames)
        │  ─ mag² saturated u16 → spec (PSRAM, ~360 KB)
        │  ─ per-half allsum (head 100-1550 Hz / tail 1550-3000 Hz) for coarse_sync
        ▼
spec_q   ← SpecBundle { spec, allsum_head, allsum_tail }  (~200 ms before SlotEnd)
slot_q   ← Slot { audio, wav_idx }                        (at SlotEnd)
        │
        │  decode_pipeline (PRO_CPU, prio 6)
        │  ┌─ 2. coarse_sync_split_with_allsum → top 30 candidates
        │  │   (head + tail in parallel via dual_core dispatch)
        │  ├─ 3a. pass2 (refine, top 15 by sync_quality_block0)
        │  │   uses Goertzel per-symbol DFT (zero scratch ← Phase 1.7.7)
        │  └─ 3b. stage3 (LLR + BP/OSD per candidate)
        │      also Goertzel for the full 21-sync + 58-data cs build
        ▼
results  → [(freq_hz, dt_sec, message77, hard_errors, snr_db)]
        │
        │  qso::QsoManager FSM
        ▼
LCD UI ← decoded_list + waterfall + status bar
BLE   ← CI-V PTT toggle when operator picks a callsign (Acoustic/Qso)
I2S TX ← synth FT8 audio (Qso mode only)
WiFi  → UDP log (every log::info! line)
```

### Why Goertzel (Phase 1.7.7-Stick)

The previous pipeline allocated 4 × 30 KB internal-DRAM `BASIS` sin/cos
scratch buffers for the per-symbol DFT (30 KB × 2 re/im × 2 dual_core
workers = 120 KB). On Qso mode the I2S bidir DMA descriptor could not
find a large-enough contiguous internal chunk and the mode failed to
init with `i2s_alloc_dma_desc: allocate DMA buffer failed`.

Phase 1.7.7 replaced the BASIS dot-product with a generalised Goertzel
recursion that holds 3 state values per (sym, tone) on the stack and
throws them away on return. Same DFT math, **zero scratch**, +0.16..
+0.63 dB SNR improvement (f32 Goertzel has more precision than the
prior Q15 BASIS dot product). M5StickS3 Qso mode now boots cleanly
with the I2S bidir DMA alloc succeeding on the first try.

See the project memory `phase177-goertzel` and PR #103 for the full
migration log.

---

## 7. UI and operation

### LCD layout (135 × 240 portrait)

```
┌─────────────────────────────────┐
│ status bar (mode / RX gain / …) │
├─────────────────────────────────┤
│                                 │
│   waterfall (200-2700 Hz band)  │
│   ~80 rows tall                 │
│                                 │
├─────────────────────────────────┤
│   decoded list (callsigns + DT) │
│   highlighted = currently       │
│   selected for response         │
├─────────────────────────────────┤
│   TX strip (next outgoing msg)  │
└─────────────────────────────────┘
```

### Buttons

| Button | Short press | Long press |
|---|---|---|
| **KEY1** (front) | Cycle selection through decoded list | Mark current slot's median DT as the new sync reference (Phase 1.7.3+) |
| **KEY2** (top) | Open / dismiss menu | Cycle `BootMode` + restart (see §5) |
| **Reset side** | Reboot | — |

### Menu (KEY2 short press)

Opens a modal menu (Phase 1.7-Stick) covering:

- **Band picker** — 11 preset HF bands (160 m → 6 m); changes the
  IC-705's frequency via BLE CI-V.
- **Mode picker** — USB / LSB / DATA-U / DATA-L.
- **Auto-CQ toggle** — when ON, the firmware emits a CQ message every
  slot while `QsoState == Idle`.
- **Auto-DF toggle** — when ON, the TX picker finds an unoccupied
  audio-DF slot (using `tx_picker::OccupancyMap` populated from the
  decoded stream) so your TX doesn't QRM another station.

---

## 8. QSO workflow (Acoustic / Qso mode)

1. Pair the IC-705 over BLE (one-time: see §9 troubleshooting if it
   doesn't pair).
2. Boot the Stick. With `auto_cq` enabled, the firmware emits CQ in
   every TX slot.
3. When a station responds, their callsign appears in the decoded
   list with a `[YOUR-CALL]` highlight (the `decode_pipeline` filter
   recognises addressed messages).
4. **KEY1 short press** to select the responder.
5. The firmware advances the QSO FSM (`Cq → InCallSelected →
   WaitingReport → WaitingR73`), keys IC-705 PTT via BLE CI-V at
   each TX slot boundary, transmits the next message.
6. After R-73 received, FSM returns to `Cq` and auto-CQ resumes.

The full state machine lives in `embedded-poc/mfsk-app-shared/src/qso.rs`.

---

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `bindgen / unable to find libclang` at build | `~/export-esp.sh` not sourced | `source ~/export-esp.sh` in the shell before `cargo` |
| Flash completes in ~5 s with "Segment … has not changed, skipping write" | Re-flashing identical ELF | Touch a source file (bump a `log::info!`) and rebuild |
| Chip drops into DOWNLOAD mode after flash (`rst:0x15 USB_UART_CHIP_RESET`) | Using `espflash monitor` directly with default reset | Always use `embedded-poc/scripts/flash-monitor.sh` |
| `i2s_alloc_dma_desc: allocate DMA buffer failed` in Qso mode | DRAM fragmentation (pre-Phase 1.7.7) | Update to firmware post Phase 1.7.7 (Goertzel migration) |
| BLE CI-V doesn't pair with IC-705 | IC-705 in CI-V mode but not advertising BLE | On IC-705: `MENU → SET → Bluetooth → Bluetooth function = ON`, also `Pairing/Reception = Pairing reception`. Then power-cycle the Stick. |
| UDP log not arriving on PC | Subnet broadcast dropped by router, or firewall | Set `pc_ip = "192.168.x.y"` (unicast to your PC) in `cfg.toml`. Open UDP 9999 in the PC firewall. On WSL2 see §4. |
| Console freezes after a few minutes | USB-CDC host disconnected (chip kept emitting `println!`) | Already fixed in firmware (commit `8b46f4e` gates `println!` on `usb_serial_jtag_is_connected`). If you see it on a fresh build, your tooling is old. |
| Decoder runs but 0 decodes | Audio level too low / too high; or wrong band on IC-705 | Watch `audio capture+tx tick: NNN B/s rx` log lines — should be 192–196 kB/s sustained for healthy 48 kHz stereo I2S RX. If silent, check mic gain (`embedded-poc/m5stack-s3-app/src/audio.rs` `mic_gain_db`) or radio output level. |

### Log capture conventions

Always log to `logs/<bin>_<tag>_$(date +%Y-%m-%d).log` so per-session
diagnostics accumulate predictably. `flash-monitor.sh` handles the
log path argument; pass it consistently.

For UDP log on the PC:

```sh
embedded-poc/scripts/udp-log-listen.sh 9999 \
    | tee logs/udp_$(date +%Y-%m-%d_%H%M).log
```

---

## 10. Reference

- [`docs/EMBEDDED.md`](EMBEDDED.md) — `mfsk-core` embedded integration reference (scalar architecture, FFT-extern contract, Goertzel per-symbol DFT, Q-format reference, `mfsk-ffi-ft8` C ABI tutorial, performance benchmark, streaming pipeline, binary footprint).
- [`docs/ROADMAP.md`](ROADMAP.md) — release milestones and Phase B-Stick / B-Core plan.
- [`embedded-poc/CLAUDE.md`](../embedded-poc/CLAUDE.md) — shared embedded toolchain notes (cross-board).
- [`embedded-poc/m5stack-s3-app/CLAUDE.md`](../embedded-poc/m5stack-s3-app/CLAUDE.md) — board-specific notes for AI agents working on this crate.
- `mfsk-core/` — the actual decoder library (host + embedded shared, published on crates.io).
- `embedded-poc/mfsk-app-shared/` — board-agnostic app logic (QSO FSM, UI primitives, WiFi, log fanout).
