# M5Stack CoreS3 receiver — user manual

One binary, four receivers, and a radio on the USB port.

This board is the project's main hardware target: unlike the M5StickS3
it can act as a USB host, so it takes audio directly from a
transceiver's USB Audio interface rather than through a microphone.
Verified on 2026-08-23 against an IC-705 on 40 m, decoding six to eight
FT8 stations per slot down to −24 dB.

For the StickS3 — the demo / acoustic board — see
[`MANUAL_M5STICKS3.md`](MANUAL_M5STICKS3.md). Board-agnostic decoder
notes live in [`EMBEDDED.md`](EMBEDDED.md).

---

## 1. What it does

Four receivers share one flashed image. Which one runs is stored in
NVS and chosen from the touch panel, so changing mode does not mean
re-flashing — which matters here, because installing the USB host
driver takes the port a flasher would use.

| Mode | What it does | Radio needed |
|---|---|---|
| `uac` | FT8 from the radio's USB Audio interface | yes |
| `wspr` | WSPR receiver — spot list, wsprnet upload | yes |
| `fst4` | FST4 wideband monitor | yes |
| `decode` | FT8 from a WAV baked into the image | no |

`decode` is the demo mode: it needs no radio and no antenna, and is
the fastest way to confirm a board works at all.

---

## 2. Hardware

### Required

- **M5Stack CoreS3** (ESP32-S3, 16 MB flash, 8 MB PSRAM). CoreS3 SE
  works too.
- **USB-C cable** to a PC for flashing.
- For everything except `decode`: a transceiver with a **USB Audio**
  interface, and a USB-C cable to it.

### The power constraint, which is not optional to understand

One USB-C connector cannot both take power in and hand it out. The
firmware therefore decides its role **once, at boot**, from whether
VBUS is present:

- **VBUS present** (plugged into a PC) → stays a USB *peripheral*. The
  battery charges, the serial console works, and the board is
  flashable. It will not talk to a radio in this state.
- **VBUS absent** (on battery) → becomes a USB *host*, powers the
  connector, and enumerates the radio.

So the sequence for live reception is: unplug from the PC, **press the
button briefly to reset**, then attach the radio. Unplugging alone does
not switch roles — the decision was made at boot.

In host mode the USB-Serial-JTAG device is gone, so the board cannot be
flashed and has no serial console. WiFi is the only channel; see §6.

### On-board devices this firmware uses

| Device | Address | Used for |
|---|---|---|
| AXP2101 | 0x34 | Power rails, battery/VBUS voltage |
| AW9523B | 0x58 | LCD backlight, touch enable, USB host VBUS |
| FT5x06 | 0x38 | Touch panel |
| BM8563 | 0x51 | Real-time clock (see §7) |

---

## 3. First-time setup

### 3.1. Toolchain

```sh
cargo install espup espflash
espup install
source ~/export-esp.sh    # needed in every shell that builds
```

### 3.2. Configure

```sh
cd embedded-poc/m5stack-cores3-app
cp cfg-sample.toml cfg.toml
$EDITOR cfg.toml          # see §4
```

`cfg.toml` is gitignored. It has never been committed and must not be —
it holds your WiFi passphrase.

### 3.3. Build and flash

```sh
source ~/export-esp.sh
cargo build --release
../scripts/flash-monitor.sh \
    target/xtensa-esp32s3-espidf/release/mfsk-core-m5stack-cores3-app \
    logs/boot_$(date +%F).log 110
```

Use the script rather than `espflash` directly. It detects the flash
size (writing the wrong one bricks the app in a way that survives
`erase-flash`), and it distinguishes a failed write from a successful
one — it exits non-zero and says which failure it was:

| exit | meaning |
|---|---|
| 2 | serial port unavailable |
| 3 | the write never completed (capture window too short) |
| 4 | wrote OK, but the chip parked in DOWNLOAD mode |

Exit 4 is recovered with a **short** press of the board's button. A
long press (~2 s) puts it *into* DOWNLOAD mode.

"Segment … has not changed, skipping write" is **not** a successful
flash — the chip still runs the previous binary. Touch a source file to
force a real write.

---

## 4. `cfg.toml` reference

Read at compile time by `build.rs`, so a change needs a rebuild.

### `[wifi]`

```toml
ssid = "your-ssid"
psk  = "your-passphrase"
pc_ip = "auto"      # or "192.168.1.9", or "255.255.255.255"
port = 9999
```

WiFi is not a debugging convenience on this board. It carries:

- **NTP**, which the FT8 slot grid needs before it can decode anything
  (§7),
- the **HTTP settings page** (WSPR callsign, band, wsprnet),
- the **UDP log**, which in host mode is the only console there is.

Omit the section, or set `ssid = ""`, to disable WiFi. The board still
boots.

`pc_ip = "auto"` computes a directed subnet broadcast from the DHCP
lease. Prefer it: access points often drop `255.255.255.255` for
power-save reasons but pass a directed broadcast through.

### `[station]`

```toml
call = "JA1ABC"
grid = "PM95"
```

Used by the FT8 QSO state machine when composing messages. **This board
cannot transmit** — CoreS3 has no PTT or TX audio path today (that work
is starting; see `embedded-poc/m5stack-cores3-app/CLAUDE.md`'s "TX/QSO
feasibility" section), so the FSM's output is display-only: it logs
what it would send next (`[QSO] …`), and nothing goes to the radio.
Setting `call`/`grid` is therefore for watching the FSM track a QSO
against real decodes, not for participating in one. **Leave both empty
for a receive-only station** — the FSM then stays idle, which is
correct, and better than identifying as somebody else even in a log
line nobody transmits.

WSPR takes its callsign from the HTTP settings page instead, not from
here, because it is changed more often than a rebuild is convenient.

### `[app]`

```toml
boot_mode = "uac"
```

A **seed**, not an override: written to NVS only when NVS has no mode
yet. The touch picker writes NVS at runtime, and a value here that
reapplied on every boot would undo that choice on the next restart.

Values: `uac` | `wspr` | `fst4` | `decode`. Omit to keep whatever NVS
holds.

### Build-time switches

Environment variables read at compile time. All default to off.

| Variable | Effect |
|---|---|
| `MFSK_CORES3_FORCE_UAC=1` | take USB host mode even with external power. Back-powers a PC; bench use only |
| `MFSK_CORES3_USB_PANEL=1` | draw the ten-line USB diagnostic panel, at the cost of decoded rows |
| `MFSK_WSPR_SYNTH=1` | fabricate a WSPR slot when no radio is attached |
| `MFSK_FST4_REPLAY=1` | replay a baked FST4 slot when no radio is attached |

The last two exist for desk work with no radio. **They are off by
default because what they produce is indistinguishable from a real
decode**: a fabricated `K1ABC` lands on the spot list every two
minutes, and a replayed slot shows the same stations forever. A
receiver with nothing to hear should say so.

Their fixtures are behind Cargo features (`wspr-golden`,
`fst4-replay`) so the bytes are not linked in when off — 1.8 MB of
image, off every flash.


---

## 5. Changing mode

Hold a finger anywhere on the screen for about **0.8 s**. An overlay
appears listing the four receivers, with `*` on the current one.

1. **Tap a mode.** The row turns green. The name stays readable — the
   confirmation step exists to let you check it.
2. **Press the bar underneath**, which reads `SWITCH TO <mode>`. It
   goes amber while pressed, then green with `SWITCHING TO <mode>`.
3. The board writes NVS and restarts into the chosen receiver.

Tapping outside the overlay dismisses it. There is 14 px of slop around
the edges, so a press that is nearly right commits rather than
dismissing.

To change mode without the panel: erase NVS, and set `boot_mode` in
`cfg.toml`.

---

## 6. Reading a board that has no console

In host mode the USB serial device does not exist. Everything below
assumes that.

### UDP log

```sh
embedded-poc/scripts/udp-log-listen.sh 9999 logs/session_$(date +%F).log
```

Start it **before** the board boots. WiFi associates several seconds
after the power and USB decisions are made, so those lines land in the
fanout's staging ring and are overwritten before a sink exists.

### `[boot-summary]`

Because of exactly that, each receiver re-emits the boot-critical state
once a log sink appears:

```text
[boot-summary] mode=UAC host_mode=true start_host: host+class driver installed OK
[boot-summary] rtc: clock from BM8563 2026-08-23 13:27:05Z
[boot-summary] AXP 0x90=0x8b status1=0x18 | AW9523 OUT0=0x33 OUT1=0x83 | bat=3889mV vbus=16373mV
[boot-summary] i2c: 34 38 40 51 58 69
```

Read it in this order:

- `host_mode` — what the firmware decided. `false` with a radio
  attached means VBUS was present at boot; unplug, reset, retry.
- `start_host` — whether the USB host driver actually installed.
- `rtc` — whether the clock came from the chip or has to wait for NTP.
- `OUT1` bit 7 and `OUT0` bits 1 and 5 — the three VBUS enables. All
  three must be high before anything can enumerate.
- `i2c` — a live scan. An output register readback tells you what was
  written; this tells you what is still answering.

### The link bar

The bottom 14 px of every screen, in every mode:

```text
USB H STREAM d1 V111 v---- b3.89 W-36
      │      │  │    │     │     └ WiFi RSSI, or `down`
      │      │  │    │     └ battery volts
      │      │  │    └ VBUS volts, `----` when the ADC cannot read it
      │      │  └ BOOST_EN / USB_OTG_EN / BUS_OUT_EN
      │      └ open USB devices
      └ CHARGE | NODEV | STREAM (green) | NOHOST (amber) | ERROR (amber)
```

`H`/`P` is host or peripheral — what the firmware chose, not whether
the driver came up, so `H NOHOST` is a fault and reads as one. `V!!!`
means the I/O expander has stopped answering.

In host mode the VBUS ADC cannot see the board's own boost output and
rails to full scale, so it shows `v----`. That is normal; battery
voltage is the number that matters there.

---

## 7. Time, and why decoding depends on it

Every receiver here anchors its slot grid to UTC. FT8 tolerates ±2.5 s
against a 15 s grid, so **a board with no clock decodes nothing** — the
waterfall fills with signal, candidates are found, and not one of them
resolves. This is the single most confusing failure mode on this
hardware, and it looks exactly like a broken decoder.

Two sources, in order:

1. **BM8563 RTC**, read in `pmic::init` before WiFi exists. Battery
   backed, so it survives a power cycle and works out of WiFi range.
2. **NTP**, which refines it and writes the result back to the RTC.

`[boot-summary]` says which one supplied the clock. On screen, the
status row shows UTC and the link bar shows `T` when the clock is set,
`-` when it is not.

A fresh board, or one whose backup cell has gone flat, reports
`BM8563 VL` and waits for NTP. After the first successful sync every
later boot starts with a clock already set.

---

## 8. Per-slot logging

Each mode prints one line per slot carrying the audio source, the
result, and the time budget.

**FT8**

```text
SLOT[7] src=uac p1=30 ready=30 defer=0 dec=7 tail_win=… slot_wait=981068us
```

`src=uac` versus `src=wav` says whether this is live audio or the baked
recording. `dec` is decodes. `slot_wait` near zero means the pipeline
was still working when the next slot arrived, and a separate
`OVER BUDGET` warning says so.

On a busy band that warning is expected rather than alarming: 15 s is
genuinely tight, the cost of the front end scales with how much signal
is present, and stations transmit in alternating periods, so the
busier of the two can run out of time and defer candidates.

**WSPR and FST4** are the opposite case. Their monitor loops are built
with large deliberate slack — a 2-minute WSPR slot against a few
seconds of work — and that slack is what absorbs a crowded slot, a WiFi
burst and the display task without dropping candidates. A low occupancy
figure there is the design working, not headroom to reclaim, and
exceeding the slot means a fault rather than a busy band:

```text
wspr_app: slot 2316 src=uac decoded 3 station(s)
wspr_app::ddc: compute occupancy 8210 ms / 120000 ms slot budget (6.8%)
fst4_app::scan: slot 4 budget — 13643 ms of 60000 ms (23%), 46357 ms spare
```

---

## 9. Troubleshooting

**Nothing enumerates; `d0` on the link bar.**
Check `host_mode` in `[boot-summary]`. `false` means VBUS was present
at boot — unplug from the PC, short-press reset, then attach the radio.

**Signals on the waterfall, `dec=0` on every slot.**
Three causes, in the order they are worth checking:

1. **The radio is on LSB.** FT8, FST4 and WSPR are always USB, on every
   band, including the ones where voice is LSB. On LSB the tone order
   is mirrored and the sync pattern cannot match — the waterfall looks
   perfect and nothing decodes.
2. **No clock.** Link bar shows `-` rather than `T`. See §7.
3. **Wrong frequency.** FT8 is 7074 kHz on 40 m; WSPR is 7038.6 kHz.
   They are not the same band segment.

**The board vanished from the PC.**
It is in host mode; the app owns the USB PHY and presents nothing.
Short-press reset while plugged in and it comes back as a peripheral.

**Audio level.** `rms` in the `uac: rx tick` line should sit around
−40 to −25 dBFS. Much lower and the radio's USB AF output level needs
raising; `clipped` above zero means it is too high.

**Under WSL**, every chip reset re-enumerates the USB device and
detaches it from Linux. `embedded-poc/scripts/wsl-attach-board.sh`
re-attaches it.

---

## 10. Where things are

| Path | What |
|---|---|
| `src/main.rs` | Boot, mode dispatch, WiFi/NTP |
| `src/display.rs` | FT8 controller screen and USB host bring-up |
| `src/apps/wspr.rs`, `src/apps/fst4.rs` | The other two receivers |
| `src/uac.rs` | USB host + UAC class driver, audio sinks |
| `src/pmic.rs` | AXP2101 + AW9523B |
| `src/rtc.rs` | BM8563 |
| `src/touch.rs` | FT5x06 |
| `embedded-poc/CLAUDE.md` | Hardware notes worth reading before changing any of the above |
