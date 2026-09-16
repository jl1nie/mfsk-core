# CoreS3 UAC bring-up — what to check, in what order

This procedure did its job on 2026-08-23 and
[#163](https://github.com/jl1nie/mfsk-core/issues/163) is closed: ten
unbroken minutes of UAC capture, 125 MB, zero errors, with WiFi
associated throughout. The log is kept at
`embedded-poc/m5stack-cores3-app/logs/uac_stream_2026-08-23.log`.

It is kept because it is the checklist to re-run whenever the USB
audio path stops working — a new radio, a new IDF version, a board
that came back from the drawer. It lived in
`docs/reference/EMBEDDED.md` until 2026-09-16, where 100 lines of
closed-issue bring-up sat between that document's reference material
and its reader.

Read `embedded-poc/CLAUDE.md`'s "USB host VBUS on CoreS3" and "Stacks,
heaps, and the space between them" before touching the board.


**This procedure did its job on 2026-08-23 and #163 is closed** — an
IC-705 enumerated through its internal hub and the FT8 controller ran
ten unbroken minutes at 192,512 B/s, zero errors. It is kept because
the WSPR and FST4 receivers still decode a baked golden slot: they
share the same `uac.rs`, so the transport is proven, but neither has
been run against a radio. Written before the first session so that it
would produce a *result* rather than an ambiguity, which is also why it
is still the right script for the second.

**Any UAC source satisfies checkpoint 1.** The criteria were written
around an IC-705 because that was the only consumer; for "does real
PCM reach the pipeline", a USB audio interface fed from a phone or PC
is easier to arrange and, if you play the golden recording into it,
gives a **known answer**: the receiver should decode the same two
stations it decodes from flash. A live antenna is checkpoint 2.

### Before plugging anything in

1. **Build with the host driver on.** It is opt-in precisely because it
   detaches the console:
   ```sh
   MFSK_FST4_APP_USB_HOST=1 cargo build --release
   ```
   There is one binary now; the receiver is chosen at boot from the NVS
   `boot_mode` (`decode` / `uac` / `wspr` / `fst4`), because changing
   mode by re-flashing means unplugging the radio the host driver is
   holding the port for.
2. **Have the UDP log listening.** `embedded-poc/scripts/udp-log-listen.sh`.
   The app logs, immediately before installing the host driver, whether
   the UDP sink is up — if it says it is not, stop: that boot will be
   silent and you will not be able to tell a crash from a success.
3. Expect the serial console to die the moment the driver installs.
   That is not a fault.

### What the logs will tell you

`uac.rs`'s reader prints one line per second carrying both halves of
the question:

```text
uac: rx tick: 192000 B/s (total … / … pkt / 0 err)
   | audio 12000 sa/s (want 12000), rms -32.4 dBFS, peak 4211, clipped 0
```

- **`B/s` near 192 000** — the transport is alive (48 kHz stereo × 2 B).
  Zero means nothing enumerated or the stream never started; well below
  190 k means dropped frames.
- **`sa/s` near 12 000** — the resampler is producing at the rate the
  decoder expects. **~11 025 means the source is 44.1 kHz**, which this
  build does not configure; **~6 000 means the device is streaming mono**
  and the stereo de-interleave is taking every other sample of one
  channel.
- **`rms`** — the half that byte counters cannot answer. A device
  happily streaming 192 kB/s of digital silence (muted source, wrong
  input selected, codec unconfigured) reads `-99.0 dBFS` here and looks
  perfect everywhere else. Real receiver noise should sit somewhere
  around −40 to −25 dBFS; a strong signal well above that.
- **`clipped`** — nonzero means the source level is too high and the
  decoder is being fed a distorted signal.

Then, per slot, the application says what it did with it:

```text
fst4_app::capture: front end … source UAC
fst4_app::scan: slot N — 50 candidates in … ms
```

`source UAC` is the flag that says the golden replay has stepped
aside. If audio stops mid-slot the capture task ends that slot short
and says so rather than hanging.

### Checkpoint 1 — real PCM reaches the decoder

Play the golden FST4-60 recording into the source. Expect
`CQ N5TM EL29` and `CQ K9KFR EN71`, at ~1100.6 and ~1330.6 Hz. Anything
else — no decodes with healthy `rms`, or decodes at the wrong
frequencies — is a real finding, not a setup problem, and the
per-second telemetry above is what tells you which.

### Checkpoint 2 — live antenna

The slot grid is anchored now. `Ft8ChunkSink` takes a rough one-time
anchor from a plausible clock (`time_sync::samples_to_next_slot_12k`,
RTC then NTP), and once NTP has disciplined the clock its UTC drift
check owns the phase and re-anchors on error past 250 ms. Until then —
or forever, off-grid — #356 rides the air: `decode_pipeline` posts
coarse sync's DT median through `set_bootstrap_slot_shift_12k` (the
top-5 candidate median before any decode, the confirmed-decode median
after) and `Ft8ChunkSink` applies it. The moment
`clock_is_disciplined()` turns true the air-sync path is skipped
entirely. Watch the log for `NTP-disciplined — UTC owns the slot
phase`, or a run of `air-sync: … → ± samples` lines converging toward
zero.

Checkpoint 1 does not care either way — the golden recording is a whole
slot and the decoder finds its own `dt` — which is why it is worth
doing first. What is still not covered: a grid more than ~1 s out at
the very first slot, since coarse sync only searches ±1 s and cannot
report a DT it cannot see (#356's air-phase-lock follow-up).

