# MfskCore — Swift bindings

A SwiftPM package wrapping `mfsk-ffi`'s C ABI, so an iOS or macOS app
can decode and synthesise FT8, FT4, FST4, WSPR, JT9, JT65, Q65 (all
ten sub-modes) and JTTY without writing its own bridging header.

```swift
import MfskCore

// TX: pack, tone, place in a slot — three ABI stages in one call.
let slot = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JL1NIE", report: "PM95",
                                       frequencyHz: 1500)

// RX: one Decoder per mode (and per thread), reused across periods. Open it
// from the mode's own defaults, changing what you want.
var params = try DecodeParams(mode: .ft8)
params.station = .init(call: "JL1NIE", grid: "PM95")
params.rxFrequencyHz = 1500
var extras = Extras()
extras.a7 = true                    // an option the mode lacks throws .unsupported
let decoder = try Decoder(mode: .ft8, params: params, extras: extras)
for row in try decoder.decode(slot, period: 1_700_000_010 / 15) {
    print(row.frequencyHz, row.snrDB, row.text)
}

// Live audio: push what the callback gives you, tell the stream what time it
// is as often as you have a reading, decode when a slot fills.
let stream = try CaptureStream(mode: .ft8, sampleRate: 48_000)
try stream.push(capturedSamples)                 // [Int16] or [Float]
try stream.setTime(utcNanoseconds: nowNanoseconds)
if let slot = try decoder.decode(stream) {
    print(slot.period, slot.decodes.map(\.text))
}
```

The same `Decoder` decodes WSPR, JT9, JT65 and Q65 (`Decoder(mode: .q65a30)`):
the ABI has one handle for every slot mode. JTTY, which has no slot, has
`JttyReceiver`; MSK144 is not decoded by this binding.

## Running the tests

```sh
bindings/swift/scripts/test.sh          # builds libmfsk, then swift test
```

98 tests (`grep -rc 'func test' Tests`; it was 86 before the one-decoder-handle
ABI and the numbers below predate it). **This package has not been built since
that port** (it was written on a machine with no Swift toolchain), so the count
and the timing are unmeasured until the first run on a Mac. The slow ones are
Q65, which decodes a 30 s period several ways, and WSPR/JT9/JT65.
CI runs exactly this script on `macos-latest` (the
`Swift binding (macOS) + iOS build` job), which is also where
`aarch64-apple-ios` is built — both need Xcode, one for XCTest and one
for the iOS SDK. The script builds `mfsk-ffi` with cargo and passes the
`-L`/`-rpath` for `target/release`, because `Package.swift` deliberately
carries no `unsafeFlags`: a package that has them cannot be used as a
dependency at all, which for a binding is fatal. On macOS it also points
`DEVELOPER_DIR` at Xcode when `xcode-select` is on the Command Line
Tools, since XCTest ships with Xcode and not with the CLT.

`Sources/CMfsk/module.modulemap` includes `mfsk-ffi/include/mfsk.h`
**in place** rather than copying it. CI already fails if that committed
header drifts from the library ("Committed headers are current"), so
referencing it means there is nothing here to keep in step.

## What this wraps

| Area | Swift | C |
|---|---|---|
| Mode introspection | `Mode`, `ModeInfo`, `Capabilities` | `mfsk_mode_count` / `_at` / `_name` / `_from_name` / `_info` / `_caps` |
| Parameters | `DecodeParams` (`init(mode:)`: band, Rx/Tx frequency, tolerance, depth, averaging, deep search, EME delay, `station`, `qso`, `ap`, `contest`) | `mfsk_params_init`, `MfskParams` |
| Library options | `Extras` (sync threshold, candidate budget, OSD, strictness, `strategy`, equalisation, a7, sniper, `apHint`, `noiseBlanker`, search window, Fano cycles, Chase trials, Q65 `pileup` / `maxDrift` / `fading`) | `mfsk_extras_init`, `MfskExtras` |
| Decoding | `Decoder(mode:params:extras:)`, `.decode(_:sampleRate:period:handler:)` for `[Int16]` / `[Float]`, `.setParams`, `.setExtras` | `mfsk_decoder_open` / `_close` / `_set_params` / `_set_extras` / `_decode_i16` / `_decode_f32` |
| Live capture | `CaptureStream` (`push`, `setTime`, `position`, `takeSlot`), `Decoder.decode(_ stream:)` | `mfsk_stream_*`, `mfsk_decoder_decode_stream` |
| Wideband IQ | `IQReceiver` (`addChannel`, `decoder(forChannel:)`, `push`, `poll`, `retune`, `setTime`, `gap`), `IQDecode`, `IQFormat` | `mfsk_iq_*`, `mfsk_iq_channel_decoder` |
| State between periods | `Decoder.addCallsign(_:)`, `.clear()`, `.setQ65Callers(_:)` | `mfsk_decoder_add_callsign` / `_clear` / `_set_q65_callers` |
| Raw FEC bits | `Decoder.informationBits(at:)` | `mfsk_decoder_copy_info` |
| Rows as they are found | `Decoder.onDecode { row in … }`, or `handler:` on one call | `mfsk_decoder_set_on_decode` |
| Decode budget | `Decoder.setBudget { … }`, `.lastBudget` | `mfsk_decoder_set_budget`, `mfsk_decoder_last_budget` |
| Transmit | `Message` (`standard` / `type1` / `freeText` / `type4`, `text(resolvedBy:)`), `Mode.synthesiseFrame`, `Mode.synthesiseSlot` | `mfsk_pack77*`, `mfsk_unpack77`, `mfsk_decoder_unpack77`, `mfsk_message_to_tones`, `mfsk_tones_to_i16` / `_f32` |
| Encoders with no tone stage | `WSPR`, `JT9`, `JT65`, `Q65` (`encode`, and `encode(…, copiedLastTx:)` for Pileup) | `mfsk_encode_wspr` / `_jt9` / `_jt65` / `_q65` / `_q65_flagged` |
| Q65 lists | `Q65History`, `Q65Callers`, `Q65SubMode`, `Q65FadingModel` | `mfsk_q65_history_*`, `mfsk_q65_callers_*` |
| JTTY | `JttyReceiver` (`push` / `poll` / `finish` / `reset` / `setParams`, `JttyParams`, `JttyUpdate`), `Jtty.tones(for:profile:)` / `.synthesise` / `.audio(for:)`, `JttyProfile` | `mfsk_jtty_*` |
| Thread pool, versions | `Runtime` | `mfsk_runtime_configure`, `mfsk_runtime_thread_count`, `mfsk_version`, `mfsk_abi_version` |
| Errors | `MfskError` (`code` + `detail`) | `MfskStatus`, `mfsk_last_error`, `mfsk_decoder_last_error` |

Three things the binding does rather than mirror:

* **Errors are thrown, and always carry a reason.** `Decoder` reads its
  own handle's error slot first and falls back to the thread-local
  global — `mfsk_decoder_copy_info` takes the handle as `const*`, so it
  can only write the global, and a wrapper that trusted the handle
  alone would report "(no detail)" for it. An option the mode does not
  have is `MfskError.Code.unsupported`, with the option named in
  `detail`, and it is thrown when the decoder is opened (or `setExtras`
  is called) rather than dropped at decode time.
* **Unset is `nil`.** The ABI spells "unset" as NaN (a frequency) or -1
  (a choice with a default) because 0 is a value; `rxFrequencyHz`,
  `toleranceHz`, `txFrequencyHz` and every `Extras` field that has a
  default are `Optional`.
* **A `Decoder` is not `Sendable`.** One per thread, like the C handle.
  Its `onDecode` handler can fire from a worker thread on a `desktop`
  build; see the doc comment on `Decoder.onDecode`.
* **`Q65SubMode` is a separate type from `Mode`**, because Q65's
  discriminants are its own and are not in slot-length order (`a15` is
  6, appended rather than inserted so the earlier numbers stayed put).
  `.mode` bridges to the `Mode` a decode row reports, and
  `Q65SubMode(someMode)` bridges back.

## The one thing worth reading before using an AP hint

`Extras.APHint` takes the **message's fields in order**, not roles:
`call1` is the first callsign field, which is `"CQ"` for a CQ message and
not the name of the transmitting station. A hint locks message bits
(0-28, 29-57, 58-73 — see `mfsk_core::msg::ap`) rather than steering a
search, so hinting the right callsigns in the wrong fields is not a
weaker hint but a wrong one. The QSO context in `DecodeParams`
(`station`, `qso`, `ap`) builds the hypotheses WSJT-X builds from
`mycall` / `hiscall`; `Extras.apHint` is the free-form one beside it and
wins when given.

## Not wrapped yet, and why

Nearly every function in `mfsk.h` is reachable from this package. The
`Sources` tree does not call `mfsk_encode_ft8` / `_ft4` / `_fst4s60` (the
staged `Message` → `Mode.synthesiseFrame` path covers them),
`mfsk_jtty_pending` (`poll()` drains the queue), `mfsk_jtty_synth_len`,
`mfsk_jtty_tones_to_f32` (`Jtty.synthesise` writes `Int16`) or
`mfsk_q65_history_record` (`Q65History.record(_:)` feeds the history a row
at a time). What is left beyond those is one struct field:

* **The two thread hooks** on `MfskRuntimeConfig`. They exist so an
  Android JNI consumer can `AttachCurrentThread` on each rayon worker;
  there is no Apple-platform equivalent to call.

Gone with the ABI's one-decoder-handle rewrite, and so gone from here:
the per-family `Q65.decode` strategies and `Q65.Params` (a Q65 `Decoder`
with `Extras` replaces them), `CallsignHashTable` (the decoder owns its
table), `DecodeSession.keepKnown` / `keepFFTCache`, the `DecodeDefaults`
introspection, and the `WSPR` / `JT9` / `JT65` decode functions.

## Using it from an app

`Package.swift` here links `libmfsk` by name and leaves the search path
to the consumer. For a real app, build the static library for the target
and link that:

```sh
cargo build -p mfsk-ffi --release --target aarch64-apple-ios \
    --no-default-features --features mobile
```

`mobile` is the feature set that drops rayon and serde — worth reading
`mfsk-ffi/Cargo.toml`'s own note on why, which is that rayon's global
pool is `num_cpus` threads with 2 MiB stacks, spawned lazily, never
joined, outside GCD's QoS, and still running when the app is
backgrounded. With it dropped, `.on_result` delivery also gets a
*stronger* contract rather than a weaker one (see
`docs/reference/STREAMING.md` §3a/§3b).

An `.xcframework` with a `binaryTarget`, so an app does not build Rust
at all, is the obvious next step and is not here yet.
