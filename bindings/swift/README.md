# MfskCore — Swift bindings

A SwiftPM package wrapping `mfsk-ffi`'s C ABI, so an iOS or macOS app
can decode and synthesise FT8, FT4, FST4, WSPR, JT9 and JT65 without
writing its own bridging header.

```swift
import MfskCore

// TX: pack, tone, place in a slot — three ABI stages in one call.
let slot = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JL1NIE", report: "PM95",
                                       frequencyHz: 1500)

// RX: one session per mode, reused across slots.
let session = try DecodeSession(mode: .ft8)
for row in try session.decode(slot) {
    print(row.frequencyHz, row.snrDB, row.text)
}

// Live audio: push what the callback gives you, decode when a slot fills.
let stream = try CaptureStream(mode: .ft8, sampleRate: 48_000)
stream.setEpoch(utcSeconds: Date().timeIntervalSince1970)
try stream.push(capturedSamples)                 // [Int16] or [Float]
if let slot = try session.decode(stream) {
    print(slot.slotStartUTC, slot.decodes.map(\.text))
}
```

## Running the tests

```sh
bindings/swift/scripts/test.sh          # builds libmfsk, then swift test
```

43 tests, ~0.4 s. CI runs exactly this script on `macos-latest` (the
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
| Mode introspection | `Mode`, `ModeInfo`, `Capabilities`, `DecodeDefaults` | `mfsk_mode_count` / `_at` / `_name` / `_from_name` / `_info` / `_caps` / `_defaults` |
| Search parameters | `DecodeParams` (+ `Depth`, `Strictness`, `Equalisation`, `APHint`) | `mfsk_decode_params_init`, `MfskDecodeParams` |
| Decoding | `DecodeSession.decode(_:sampleRate:params:)` for `[Int16]` / `[Float]` | `mfsk_session_open` / `_decode_i16` / `_decode_f32` / `_close` |
| Live capture | `CaptureStream`, `DecodeSession.decode(_ stream:)` | `mfsk_stream_*`, `mfsk_session_decode_stream` |
| Hashed callsigns | `DecodeSession.addCallsign(_:)` | `mfsk_session_add_callsign` |
| Raw FEC bits | `DecodeSession.informationBits(at:)` | `mfsk_session_copy_info` |
| Transmit | `Message` (`standard` / `type1` / `freeText` / `type4`), `Mode.synthesiseFrame`, `Mode.synthesiseSlot` | `mfsk_pack77*`, `mfsk_unpack77`, `mfsk_message_to_tones`, `mfsk_tones_to_i16` / `_f32` |
| Handle-less modes | `WSPR`, `JT9`, `JT65` | `mfsk_encode_wspr` / `_jt9` / `_jt65`, `mfsk_wspr_decode`, `mfsk_jt9_decode_at`, `mfsk_jt65_decode_at` |
| Thread pool, versions | `Runtime` | `mfsk_runtime_configure`, `mfsk_runtime_thread_count`, `mfsk_version`, `mfsk_abi_version` |
| Errors | `MfskError` (`code` + `detail`) | `MfskStatus`, `mfsk_last_error`, `mfsk_session_last_error` |

Two things the binding does rather than mirror:

* **Errors are thrown, and always carry a reason.** `DecodeSession`
  reads its own handle's error slot first and falls back to the
  thread-local global — `mfsk_session_copy_info` takes the handle as
  `const*`, so it can only write the global, and a wrapper that trusted
  the handle alone would report "(no detail)" for it.
* **`frequencyHintHz` is an `Optional`.** The ABI spells "unset" as NaN
  because 0 Hz is a frequency; `nil` is the Swift word for that.

## Not wrapped yet, and why

* **Q65 — the next slice, and no longer blocked.** Its four decode
  entry points take a sub-mode discriminant that used to be unreachable
  from the header: `MfskQ65SubMode` is mentioned by no signature, since
  every `mfsk_q65_*` function takes it as `uint32_t` on purpose, so
  cbindgen never emitted it and a wrapper would have had to hardcode
  0…9. `cbindgen.toml` asks for it by name now, so the enum and
  `MfskQ65FadingModel` are both declared in `mfsk.h` — wrapping the
  family here is ordinary work rather than a decision about numbering.
  It also wants `MfskCallsignHashTable`, which only Q65 takes.
* **`mfsk_session_set_on_decode`.** Rows delivered as they are found,
  which only changes anything for a UI wanting partial results during a
  long slot. It needs a retained box and a trampoline; worth doing when
  something asks for it.
* **The two thread hooks** on `MfskRuntimeConfig`. They exist so an
  Android JNI consumer can `AttachCurrentThread` on each rayon worker;
  there is no Apple-platform equivalent to call.

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
