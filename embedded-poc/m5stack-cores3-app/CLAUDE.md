# m5stack-cores3-app — agent notes

The project's main hardware target, and the only board here that can
host a radio over USB. Read `embedded-poc/CLAUDE.md` first — the VBUS
bit table, the stack-versus-heap trap and the AW9523B pin names all
live there and all apply to this crate. The user-facing manual is
`docs/reference/MANUAL_M5STACK_CORES3.md`; do not duplicate it here.

## One binary, four receivers

`main` reads NVS `boot_mode` and dispatches. Only the mode that booted
allocates its decode scratch (`worker_arena`), which is why the image
carries 86 KB of static DRAM rather than the ~173 KB a naive merge
would cost.

| mode | entry | audio |
|---|---|---|
| `uac` | `display::run_log_panel` → `decode_pipeline` | UAC |
| `wspr` | `apps::wspr::run` | UAC |
| `fst4` | `apps::fst4::run` | UAC |
| `decode` | `decode_pipeline::decode` | baked WAV |

`apps::{wspr,fst4}::run` take the peripherals from `main` and never
return. They were separate `[[bin]]`s until 2026-08-23; changing mode
then meant re-flashing, which on this board means unplugging the radio,
because `usb_host_install` takes the port a flasher would use.

## Things that will cost you a session if you do not know them

**Host or peripheral is decided once, at boot, from VBUS.** A board
plugged into a PC stays a peripheral — it charges and stays flashable
and will not talk to a radio. Unplugging afterwards changes nothing;
the decision was made. Every "the radio does not enumerate" report
starts by checking `host_mode` in `[boot-summary]`.

**A receiver with no clock decodes nothing — until it hears one.**
`Ft8ChunkSink` anchors its 15 s grid with
`time_sync::samples_to_next_slot_12k`, which returns `None` until the
system clock is plausible. The clock comes from the BM8563 (`rtc.rs`,
read in `pmic::init`) and is refined by NTP; unanchored, the grid used
to free-run at a phase uniform over 15 s against a mode that tolerates
±2.5 s, which produced a full evening of "the decoder is broken" on
2026-08-23. Since #356, until NTP disciplines the clock the grid pulls
itself onto the air instead: `decode_pipeline` posts coarse sync's DT
median (`bootstrap_dt_median` before any decode, the confirmed-decode
median after) through `set_bootstrap_slot_shift_12k`, and
`Ft8ChunkSink` applies it. `Ft8ChunkSink` still takes a rough one-time
anchor from a plausible RTC (to get inside the ±1 s coarse search),
but only `clock_is_disciplined()` (NTP) hands the phase to its UTC
drift check — and the whole air-sync path in `decode_pipeline` is
gated off the moment that turns true, so it costs the steady-state
slot nothing. The `wav` source is left alone.

**The boot-critical logs do not reach the only console that works.**
WiFi associates seconds after the power and USB decisions are made, and
the fanout's staging ring — small on purpose, it was eating the audio
transfer buffer — has been overwritten by then. `[boot-summary]`
re-emits mode, host decision, `start_host` result, live AXP2101/AW9523B
registers, an I2C scan and the RTC source once a sink exists. It found
a two-hour bug on the first boot after it was added. Add to it rather
than adding another one-shot log line.

**A register readback is a claim about the past.** `power_state()`
returns what `enable_usb_host_vbus` wrote; the display re-reads at 1 Hz
(`pmic::refresh_power_state`) and shows `V!!!` when the expander stops
answering. Do not surface a cached value as a live one.

## Crate-specific knobs

| env | effect |
|---|---|
| `MFSK_CORES3_FORCE_UAC=1` | take USB host mode even with external power. Back-powers a PC; for bench use only |
| `MFSK_CORES3_USB_PANEL=1` | draw the ten-line USB diagnostic panel. Off by default — it covers the decodes, and the link bar carries what an operator needs |
| `MFSK_WSPR_SYNTH=1` | fabricate a WSPR slot when no radio is attached. Off by default: it puts `DDC_TEST_CALL` on the spot list every two minutes, indistinguishable from a real decode |
| `MFSK_FST4_REPLAY=1` | replay a baked FST4 slot when no radio is attached. Same reasoning |

Cargo features `wspr-golden` and `fst4-replay` link the fixtures those
two read. Off by default, which is 1.8 MB of image: a receiver taking
audio from a radio never reads either.

`cfg.toml` (gitignored, never committed) carries `[wifi]`, `[station]`
and `[app] boot_mode`. `boot_mode` is a **seed**, written only when NVS
has none: the touch picker writes NVS at runtime and a value that
reapplied every boot would undo it.

## Gotchas

- **`opt-level = 1`** is fixed for the Xtensa LLVM regression, so
  `.collect()` temporaries are not folded. A 13.5 KB `heapless::Vec`
  built on the stack becomes 27 KB. Box the large snapshots.
- **PSRAM task stacks cannot write flash.** `xTaskCreatePinnedToCoreWithCaps(…,
  MALLOC_CAP_SPIRAM)` gives the WSPR/FST4 display tasks their stacks,
  and a flash write disables the cache that maps PSRAM, so NVS from
  those tasks aborts. `boot_mode::commit_and_restart` does it from a
  short-lived internal-stack task.
- **AXP2101 register 0x90 survives a reset.** Read-modify-write only
  ever sets bits, so one boot that wrote M5GFX's `0xBF` leaves every
  later boot inheriting it. `pmic::init` writes it whole.
- **`src/bin/lcd_minimal.rs`** pulls `pmic.rs` in by `#[path]`, so
  anything `pmic` newly depends on has to be declared there too.
- **Slot-budget logs mean opposite things per mode.** FT8's 15 s slot
  genuinely runs out on a busy band — that is the operating limit.
  WSPR and FST4 monitor loops are built with deliberate slack, so
  exceeding the slot there is a fault. Do not carry one framing across.

## FT4 live audio — the experiment that has not been run (2026-09-02)

**Every FT4 number in this repo comes from a baked replay slot, not
from a radio.** `apps/ft4.rs` registers `Ft4Sink` with
`uac::set_audio_sink`, but until 2026-09-02 `display.rs` set
`host_mode = mode == BootMode::Uac`, so in FT4 mode the VBUS boost was
never enabled and `usb_host_install()` never ran — the sink existed
and nothing fed it, and the receiver fell back to
`ft4_golden_audio.bin` every slot without that being wrong anywhere.
`host_mode` now includes `BootMode::Ft4`; the run itself is still
outstanding.

### Powering it

The obstacle was never the code. One USB-C connector cannot both take
power in and hand it out, and sourcing VBUS into a PC that is already
sourcing it browns the board out — that cost a bench session and the
ability to re-flash (#163). **The DIN Base solves it**: the M5Bus
feeds the board, so the USB-C port is free to source VBUS to the
radio.

Watch for one thing. If the AXP2101 reports the M5Bus supply as
VBUS-present, `display.rs`'s safety check refuses host mode and says
so:

```text
external USB power detected — staying a peripheral so the battery charges …
```

`MFSK_CORES3_FORCE_UAC=1` is the documented way past it, and its own
doc comment names exactly this case ("for a board fed from M5Bus
rather than the USB-C connector, where the two supplies do not
collide").

### Running it

```sh
cd embedded-poc/m5stack-cores3-app
source ~/export-esp.sh
# MFSK_FT4_REPLAY=0 so a decode cannot have come from the recording.
MFSK_FT4_REPLAY=0 cargo build --release --features ft4
../scripts/capture.sh target/xtensa-esp32s3-espidf/release/mfsk-core-m5stack-cores3-app     logs/ft4_live_$(date +%Y-%m-%d).log 240 "uac: rx tick"
```

Boot mode is in NVS; the picker is a ~0.8 s press anywhere on the
panel. Re-flashing a running app needs a 2 s RST hold, because the USB
host driver takes the console with it.

### What to read in the log, in order

1. `host` and not `periph` in the status line — otherwise the check
   above fired and the rest of the log means nothing.
2. `uac: rx tick: 192000 B/s … audio 12000 sa/s … rms -XX dBFS`.
   ~11 025 sa/s means the source is 44.1 kHz; ~6 000 means it is
   streaming mono. `rms -99.0` with healthy byte counts is digital
   silence — a muted or unselected input, which looks perfect
   everywhere else.
3. `source UAC` on the slot line, which is what says the replay has
   stepped aside.
4. Decodes. Against the replay's 11-on-a-14-signal-scene, a real 40 m
   band will give fewer and they will change every slot.

### Slot-grid alignment (#354, landed — verify on hardware)

FT4's capture window closes at 6.775 s of the slot, so a grid off by
more than a second cuts the frame rather than shifting it. The grid is
now anchored: `slot_loop` calls `SlotAccum::anchor_or_reanchor` with
`time_sync::samples_to_next_slot_12k_ms(7_500)` on the first live
block, re-anchors when the phase drifts past 100 ms (the NTP step), and
trims the residual with the DT median of each slot's decodes. Watch the
log for `slot grid anchored to UTC` then `DT median … grid …` lines
settling toward zero over the first few decoded slots.

**Not covered**: a cold start with no clock *and* no decodes — FT4's
coarse stage returns `dt = 0`, so there is no `bootstrap_dt_median`
cold-start path the way FT8 has. That is #356 (phase off the air). A
*failure* to decode on a hilltop with no NTP is still not evidence
about the decoder.

## Status (2026-08-23)

Live reception verified against an IC-705 on 40 m. **FT8**: six to
eight decodes per slot, +8 to −24 dB, callsigns and grids consistent.
**WSPR**: `slot 1 src=uac decoded 1 station(s)` — enumeration, audio
and decode, after the memory work below. USB host and audio transport
verified in all three radio modes. Not yet confirmed: FST4 producing
decodes from live audio over a full slot.

WSPR took four fixes to get there, and each was a thing one of the
other two receivers already did: the scan task could not get its stack
(the decode path was keeping three 10 368 B `IsQs` alive at once, so
both it and the worker arena were sized against a peak 20 KB larger
than necessary); `start_host` ran before the boost had ramped; the
sequence around it was not shared, so `esp_log_bridge` never reached
this mode and `EXT_HUB: ESP_ERR_NO_MEM` was invisible; and the display
task held 32 KiB of internal DRAM in a stack FST4 keeps in PSRAM.

**#357** — FT8's per-slot decode cost alternates ~2× by TX period: one
period `coarse` ~100 ms / decodes 4–8, the other `coarse` ~180 ms /
overruns the boundary 0.6–0.9 s / defers 8–11 candidates it drops /
decodes 0–2. `run_speculative_slot` had no time bound at all — a slow
slot ran every committed candidate and the overrun cascaded (a late
decode delays the next SpecBundle → blocks stage1_inc on a full
`spec_q` → stalls the UAC reader).

**The embedded FT8 decode is already the lean one**: `stage3_split` →
`process_candidates_with_ap` is single-pass, `DecodeDepth::EMBEDDED`
(OSD off), no SIC (never touches raw audio), no AP. The 7-on-qso3_busy
vs host's ~18 is the cost of that, and the right trade for the field —
the phantom bugs this suite shipped were all in the subtraction paths
this config does not use. Not a recall problem to solve.

**The fix is the deadline** (`DecodeConfig::budget_ms`, `3425da3`).
Default **2000 ms**, from the qso3_busy sweep (`logs/ft8_357_bud*`,
2026-09-04): stage 3 measures ~985 ms, `dec` is flat at 7 down to
800 ms (the deadline sheds only the doomed tail), 5 at 600 ms, 2–3 at
400 ms. `MFSK_FT8_BUDGET_MS=` overrides; `MFSK_CORES3_FORCE_MODE=decode`
runs the sweep without erasing NVS. Still needs the live-radio
confirmation that it stops the slow period stealing the next slot.
