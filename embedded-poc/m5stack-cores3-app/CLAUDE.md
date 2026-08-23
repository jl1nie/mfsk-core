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

**A receiver with no clock decodes nothing.** `Ft8ChunkSink` anchors
its 15 s grid with `time_sync::samples_to_next_slot_12k`, which returns
`None` until the system clock is plausible. Unanchored, the grid
free-runs at a phase uniform over 15 s against a mode that tolerates
±2.5 s. The clock comes from the BM8563 (`rtc.rs`, read in
`pmic::init`) and is refined by NTP. This produced a full evening of
"the decoder is broken" on 2026-08-23.

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

Open: FT8 `coarse` alternates cleanly between ~100 ms and ~180 ms with
slot parity and the slow side exhausts its budget. Cause not
established — occupancy differing between FT8's two transmit periods
explains most of it but not all, and settling it needs a measurement
(reduce the work until both periods fit, see whether the slow side's
decode count rises), not another hypothesis. See memory
`project_cores3_ft8_live_rx_verified`.
