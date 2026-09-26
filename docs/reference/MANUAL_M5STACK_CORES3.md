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
| `ft4` | FT4 from the radio's USB Audio interface | yes |
| `wspr` | WSPR receiver, with wsprnet upload | yes |
| `fst4` | FST4 wideband monitor | yes |
| `decode` | FT8 from a WAV baked into the image | no |

`decode` is the demo mode: it needs no radio and no antenna, and is
the fastest way to confirm a board works at all.

**Every mode shows the same screen** — the FT8 one: status bar,
waterfall, station list, link bar, and the menu over the top. WSPR
and FST4 had spot-list screens of their own until 2026-09-21.

- **The waterfall is drawn from the audio itself**, the same way in
  every mode, at 6 rows a second (about 17 s on screen) over 200-3 000 Hz. Its horizontal
  rules mark the slot boundaries of the running mode (7.5 s, 15 s,
  60 s or 120 s) by the board's clock — the clock's boundary, not
  necessarily the one the FT8 decoder locked onto when it has only
  the RTC and is following the air.
- **A station is green while it was heard within the last slot
  period**, then turns white on its own, whether or not the next slot
  decodes anything. The screen decides this from the time, not the
  receiver.

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

**For a bare CoreS3, the battery is the session length — and the fix
is a powered base, not a shorter session.** The rule above only says
the USB-C port must not have VBUS on it. Power arriving anywhere
*else* is invisible to that decision, so a base feeding the bottom
connector — a DIN Base, for instance — leaves the board in host mode
and running indefinitely. That is the setup for anything unattended.

On battery alone the limit is real, because in host mode the board is
*sourcing* 5 V to the radio rather than drawing any. One measured
session (2026-09-21, IC-705 on 40 m, WiFi associated, FT8 decoding
every slot, starting at `bat=3714mV`) ran **40 minutes, 160 slots**,
then stopped mid-log with no warning and no error. The line before the
cut showed free internal DRAM unchanged and a normal 7-decode slot:
the battery, not a fault. 3714 mV is a partly-charged cell — a full
pack reads about 4200 mV and lasts longer by an amount nobody has
measured.

One more thing that cost time when it happened: **a board that goes
quiet is not necessarily broken.** Its own `coredump:` line on the
next boot separates the cases — `none stored — last boot was clean`
after a power cutoff, a stored dump after a crash. Read that before
investigating anything else.

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
decode**: a fabricated `K1ABC` lands on the station list every two
minutes, and a replayed slot shows the same stations forever. A
receiver with nothing to hear should say so.

Their fixtures are behind Cargo features (`wspr-golden`,
`fst4-replay`) so the bytes are not linked in when off — 1.8 MB of
image, off every flash.


---

## 5. The menu — mode and settings

Hold a finger anywhere on the screen for about **0.3 s**. The moment a
finger lands, a 2 px border is drawn round the whole screen — no
animation. Border means the panel felt it; no border means the touch is
not reaching it. That is all it is there to say.

The overlay opens on a four-row **root**:

| Row | What it holds |
|---|---|
| `MODE` | which receiver boots (FT8, FT4, WSPR, FST4) |
| `CONFIG` | how the slot phase is kept (NTP, or the air's DT), and whether WiFi comes up |
| `FREQ` | the running receiver's dial presets, sent to the radio over CAT (see below) |
| `DEMO` | running without a radio (`WAV REPLAY` — a recording, decoded on a loop) |

Pages have different numbers of rows. **Pressing an unused band does
nothing** — it does not dismiss. Only a press outside the widget does.

A root row only navigates — opening a page needs no confirmation. Inside
a page it works as it always has: **select, then commit**.

1. **Tap a row.** It turns green. The name stays readable — the
   confirmation step exists to let you check it.
2. **Press the bar underneath**, which reads `APPLY <what you picked>`.
   It goes amber while pressed, then green with `APPLYING …`.
3. The board writes NVS and restarts (both kinds of setting restart; `FREQ`, below, is the exception).

The setting this boot is running carries a `*`. Tapping outside
dismisses the overlay, and the next open starts at the root again — an
overlay that reappears on a page nobody chose is how a stray tap lands
on a setting. There is 14 px of slop around the edges, so a press that
is nearly right commits rather than dismissing.

### CONFIG — the slot phase, and the radio

Two settings, four rows. They are unrelated to each other, so each
carries its own `*`: one marks the time source this boot is using, the
other marks whether WiFi was brought up.

#### The slot phase

**This does not choose a time source.** The clock belongs to the log:
QSO logging needs the minute right, that comes from NTP beforehand, and
the RTC holds it for weeks. What this chooses is how the *slot phase* —
which drifts on its own — is kept.

| Row | Meaning |
|---|---|
| `TIME: NTP` | take NTP at boot; UTC owns the slot phase. The default |
| `TIME: AIR DT` | do not start NTP; the phase comes from the air, the clock stays the RTC's (for the log) |

What drifts: the ESP crystal at −3.3 ppm, about 11.9 ms an hour. A few
days away from a network makes that **seconds**, against FT8's ±1.0 s
coarse search and a usable plateau of about ±0.4 s. `AIR DT` corrects
that from the air's DT, and leaves the clock alone — a second of clock
error costs a minute-resolution log nothing.

There is no reason to choose `AIR DT` where NTP is reachable: an
acquisition costs a 25 s capture and a dozen seconds of arithmetic,
where NTP gives the same phase in a few seconds.

**It does not acquire on every boot.** A successful acquisition is
written to NVS (`grid_fix`: the sub-second phase, when it was taken, and
its confidence), and the next boot restores the grid from **the RTC's
seconds plus that stored fraction**. The stored value is relative to the
RTC's own grid, so a constant RTC error cancels out and only drift
between sessions counts. A fix older than 12 hours (≈0.14 s of
holdover) or weaker than 0.55 is not used, and the board acquires
instead. If the seed is wrong anyway, three slots with nothing decoded
re-acquire from the air — that search covers the whole 15 s period, so
it recovers **whole seconds of error** as well as the fraction.

#### WiFi

| Row | Meaning |
|---|---|
| `WIFI: ON` | associate at boot, as every build did before this setting. The default |
| `WIFI: OFF` | leave the radio down for this boot |

**Why this is its own setting and not a consequence of `TIME: AIR DT`.**
The two answer different questions: the time source says where the
*phase* comes from, this says whether there is a *console*. A hilltop
with a phone hotspot wants `AIR DT` and a log both, so one setting
cannot stand for the other.

What `WIFI: OFF` buys, on a hilltop where the configured AP is not
there: the association campaign is four attempts three seconds apart,
and while the driver hunts for an AP it cannot find it runs at FreeRTOS
priority 23 — above anything this app creates. Measured at ~40 % of
decoder throughput (`fst4_sync_search` 711 → 1 395 ms per candidate),
and it lands on the first slots after boot, which are exactly the ones
a cold `AIR DT` acquisition needs.

What it costs. In FT8 (UAC) mode the USB host driver has taken
USB-Serial-JTAG, so **WiFi is the only console**: the UDP log and the
HTTP config page both go with it, and a board that fails on a hilltop
shows nothing but its panel. It also takes NTP (so `TIME: NTP` has
nothing to sync from and the grid falls back to the RTC), and in WSPR
mode it takes the wsprnet upload, which is most of what a WSPR receiver
is for.

The way back is the panel: `WIFI: OFF` does not disable the mode
picker, so the same three presses turn it on again.

To change mode without the panel: erase NVS, and set `boot_mode` in
`cfg.toml`.

### FREQ — the dial

`FREQ` is on the root rather than under CONFIG because CONFIG already
fills all four rows, and changing the dial is the one setting an
operator makes in the field.

It lists the **running receiver's** dial presets and nobody else's: an
FT8 board has no use for a WSPR dial.

| Receiver | Presets (dial, MHz) |
|---|---|
| FT8 | 160 m 1.908 (`JA`), 80 m 3.531 (`JA`), 80 m 3.573, 40 m 7.041 (`JA`), 40 m 7.074, 30 m 10.136, 20 m 14.074, 17 m 18.100, 15 m 21.074, 12 m 24.915, 10 m 28.074, 6 m 50.313, 2 m 144.460 (`JA`) |
| FT4 | 80 m 3.575, 40 m 7.0475, 30 m 10.140, 20 m 14.080, 17 m 18.104, 15 m 21.140, 12 m 24.919, 10 m 28.180, 6 m 50.318 |
| WSPR | 160 m 1.8366, 80 m 3.5686, 60 m 5.3647, 40 m 7.0386, 30 m 10.1387, 20 m 14.0956, 17 m 18.1046, 15 m 21.0946, 12 m 24.9246, 10 m 28.1246, 6 m 50.293, 2 m 144.489 |
| FST4 | none — the bar reads `no presets for this mode` |

The FT8 table carries the JA channels beside the IARU ones, labelled
`JA` so an operator abroad can tell them from the IARU channel on the
same band. The IARU FT8 channels and the whole FT4 table are the ones
WSJT-X itself lists; the WSPR table is the band list the WSPR receiver
already uses. FST4 has no table yet: WSJT-X's defaults for it are LF/MF,
which the IC-705 does not transmit on, and no HF channel has been agreed.

The widget has four rows. A table that fits is shown whole; a longer one
gives the last row to `NEXT >` and shows three presets per page (FT8
five pages, FT4 three, WSPR four), and the bar underneath reads
`pick a dial  1/5`. `NEXT >` past the last page wraps to the first, and
tapping it turns the page — it is not a selection. Opening `FREQ` lands
on the page that holds the radio's dial when that dial is one of the
presets. That row carries the `*`: it marks the dial the radio last
reported.

Then it works as the other pages do — tap a row, press the bar, which
reads `APPLY <what you picked>` — with two differences. **It does not
restart the board**: the overlay closes at once and the receiver carries
on, since a wrong dial costs one more tap rather than a reboot. And what
it writes is the radio, not NVS-then-reboot:

- `05` sets the frequency, then `26 00 01 01 01` sets USB, data on,
  filter 1 (what WSJT-X asks for as "Data/Pkt" on an IC-705).
- The board then reads the dial back. Transceive only reports a dial the
  radio *moved* to, so this is what makes a refused `05` show as the old
  dial in the status bar rather than the requested one.
- The dial is saved to NVS as `rig_hz` and **sent again the next time the
  radio connects** — after a reboot the board puts the radio back on the
  last dial applied here, USB-D FIL1 included, whatever it was left on.
  It is saved once the radio has been sent it, and not again while it is
  unchanged.

The command goes out within 0.2 s of the press. With no radio connected
the page still lists the presets and can be applied, but there is no `*`
(no dial has been reported), and the dial is held rather than
lost: the newest choice is sent when the radio connects, and nothing is
written to NVS until then. CAT only exists in host mode, so on a board
that did not come up as a USB host nothing ever sends it.

### CAT — the radio's USB CI-V port

CAT is how the board reads the radio's dial, and sets it from `FREQ`. It
starts with the USB host and needs nothing configured on the board.

**What it needs.**

- The IC-705, on the same USB-C cable that carries the audio. It is one
  composite device (VID `0c26`, PID `0036`) with the audio interfaces and
  **two** CDC-ACM serial functions; CI-V is the first, which the radio
  calls "USB (A)". The firmware opens that function's **data
  interface (interface 1)** directly, not its control interface.
- CI-V address **`A4`**, the IC-705's factory default (not `94`, which is
  the IC-7300's). The firmware has no setting for another one and drops
  frames from any other address.
- Audio first. The CAT task waits (up to 60 s) until the audio input
  stream has enumerated before it opens the port, so a shortage of USB
  resources can cost CAT and never the receiver.

**Why the data interface.** The ESP32-S3's USB host has eight channels,
one per open pipe, and the IC-705 already takes seven of them. Opening
the control interface added a notification pipe beside the two bulk
ones; the radio's audio interface then failed to enumerate ("No more HCD
channels available") and the board restarted about every 33 s. On the
data interface the driver allocates only the two bulk pipes, and audio
held 0 errors and FT8 7 decodes per slot with it open.

**What it reads.** On connect, the dial (`03`) and the mode (`26 00`).
After that, the radio's transceive frames: the dial the status bar shows
follows the radio's knob. That same dial is where `all.txt` and the ADIF
log take their frequency from. The mode is only written to
the log; CAT does not fill the status bar's mode field, which reads
`---`. When the radio is unplugged the task closes the port, waits, and
opens it again when the radio returns.

**What it writes.** Only the `FREQ` page's commit (above). Nothing else
is ever sent: no PTT and no keying, so the CoreS3 remains a receiver.

**What it does not touch.** DTR and RTS are never asserted. The IC-705's
"USB SEND" and "USB Keying" settings can map either line to PTT or CW
keying, and the driver leaves both alone unless asked; opening the data
interface means it offers no line-control calls on the handle at all.
CAT carries a dial and a mode and nothing about time: the clock and the
slot phase (CONFIG above, §7) do not depend on it.

**Not yet confirmed on hardware.** The port choice and the frame
exchange were probed on the IC-705 on 2026-09-23 with a read-only build:
`03` answered 7.041 MHz, `26 00` answered USB-D FIL1, transceive
frames followed the dial, and the radio's USB echo-back was off. The CAT
task and the `FREQ` page themselves have not been run against the radio
yet. Whether any IC-705 CI-V menu setting has to be changed for this to
work is not settled either; the sources do not record one.

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

**The dial in the status bar reads `----`, or `FREQ` shows no `*`.**
No dial has been reported over CAT. CAT exists only in host mode, opens
only after the audio input has enumerated, and accepts only replies from
CI-V address `A4`. Look for `cat: IC-705 CI-V open (USB A)` in the UDP
log; if it never appears, the port was not opened.

**`FREQ` was applied and the status bar still shows the old dial.**
The radio refused the frequency command. The log carries
`cat: rig refused a command (NG)`, and the dial the board reads back
after every apply is the old one.

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
| `src/civ_usb.rs` | CAT: the IC-705's CI-V over USB |
| `src/pmic.rs` | AXP2101 + AW9523B |
| `src/rtc.rs` | BM8563 |
| `src/touch.rs` | FT5x06 |
| `embedded-poc/CLAUDE.md` | Hardware notes worth reading before changing any of the above |
