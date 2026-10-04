# mfsk-ffi

C ABI wrapper around [`mfsk-core`](https://crates.io/crates/mfsk-core) that
exposes the WSJT-family decoders and synthesisers to C, C++, and JNI
(Android) consumers. Not published to crates.io, but every tagged release
attaches a prebuilt `linux-x86_64` tarball (library + `mfsk.h`) on the
[GitHub Releases](https://github.com/jl1nie/mfsk-core/releases) page —
other platforms/ABIs (including Android) still need a local
`cargo build -p mfsk-ffi`.

> **Embedded (no_std + alloc) targets — ESP32-S3, RP2350, Cortex-M:**
> build `mfsk-core` directly with `alloc,ft8,fft-extern` — an ESP-IDF
> project needs a Rust staticlib shim for the FFT-planner symbol either
> way, and once that shim exists the C ABI adds nothing.
> `mfsk-ffi` (this crate) is the desktop/mobile-focused C ABI covering every
> `MfskMode` (26 of them: FT8, FT4, the five FST4 sub-modes, WSPR, JT9,
> JT65, the ten Q65 sub-modes, MSK144, JTTY and uvpacket's four
> profiles); the FT8-only `mfsk-ffi-ft8` embedded crate that used to sit
> beside it is retired.

## Build

```
cargo build -p mfsk-ffi --release
```

Produces:

| File                                  | Purpose                            |
|---------------------------------------|------------------------------------|
| `target/release/libmfsk.{so,dylib}`   | Shared library (`cdylib`)          |
| `target/release/libmfsk.a`            | Static library (`staticlib`)       |
| `mfsk-ffi/include/mfsk.h`             | cbindgen-generated C header        |

The header is regenerated on every build. It is committed to the repo
so consumers who only need the header (e.g. to write JNI bindings
without building Rust) can grab it directly from `main`.

## Linking

**C (gcc / clang):**

```
gcc your_app.c -I mfsk-ffi/include \
    -L target/release -lmfsk -lpthread -lm -ldl \
    -o your_app
```

**C++ (same, with `-std=c++17`):**

```
g++ -std=c++17 your_app.cpp -I mfsk-ffi/include \
    -L target/release -lmfsk -lpthread -lm -ldl \
    -o your_app
```

**Android (NDK cross-build)** — build per-ABI with cargo-ndk and
bundle the resulting `libmfsk.so` into `app/src/main/jniLibs/<abi>/`:

```
cargo install cargo-ndk
cargo ndk -t arm64-v8a -t armeabi-v7a -t x86_64 \
    build -p mfsk-ffi --release
```

See `../bindings/kotlin/` for the maintained binding (Kotlin wrapper + JNI
C shim + build instructions for both desktop-JVM testing and Android).

**Apple (iOS / macOS):** `../bindings/swift/` is a SwiftPM package over
this header — `import MfskCore`, then `Decoder(mode: .ft8)`. For a
device build, `cargo build -p mfsk-ffi --release --target
aarch64-apple-ios --no-default-features --features mobile`; `mobile` is
the feature set that drops rayon, which is what keeps an unattached pool
of `num_cpus` threads out of a backgrounded app.

## Quick start (C++)

Encode → decode, all the way through the ABI. **Nothing here is freed**:
every buffer belongs to the caller, which is the property the v2 surface
was rebuilt for.

```cpp
#include "mfsk.h"
#include <cstdio>
#include <cstring>
#include <vector>

int main() {
    // 1. Synthesise "CQ JA1ABC PM95" at 1500 Hz, in the three stages —
    //    pack, tone, render — each into a buffer sized from the mode.
    uint8_t msg[77];
    if (mfsk_pack77("CQ", "JA1ABC", "PM95", msg) != MFSK_STATUS_OK) {
        fprintf(stderr, "pack failed: %s\n", mfsk_last_error());
        return 1;
    }
    std::vector<uint8_t> tones(mfsk_symbol_count(MFSK_MODE_FT8));
    size_t nTones = 0;
    mfsk_message_to_tones(MFSK_MODE_FT8, msg, tones.data(), tones.size(), &nTones);

    std::vector<int16_t> frame(mfsk_synth_output_len(MFSK_MODE_FT8));
    size_t nPcm = 0;
    mfsk_tones_to_i16(MFSK_MODE_FT8, tones.data(), nTones, 1500.0f, 8000,
                      frame.data(), frame.size(), &nPcm);

    // 2. Place it in a full slot at the mode's own TX offset — ask,
    //    rather than assume: FST4's five sub-modes differ here.
    MfskModeInfo info;
    memset(&info, 0, sizeof info);
    info.size = sizeof info;
    mfsk_mode_info(MFSK_MODE_FT8, &info);
    std::vector<int16_t> slot(info.slot_samples_12k, 0);
    const size_t at = static_cast<size_t>(info.tx_start_offset_s * 12000.0f);
    for (size_t i = 0; i < nPcm && at + i < slot.size(); ++i) slot[at + i] = frame[i];

    // 3. Decode it. `mfsk_params_init` writes the mode's defaults
    //    (WSJT-X's parameter block); pass NULL to open for the same, or
    //    adjust the fields you care about first. The second NULL is
    //    `MfskExtras`, the library's own options, unset.
    MfskParams p;
    memset(&p, 0, sizeof p);
    p.size = sizeof p;
    mfsk_params_init(MFSK_MODE_FT8, &p);

    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskDecoder* d = mfsk_decoder_open(MFSK_MODE_FT8, &p, nullptr, &st);
    if (d == nullptr) {
        fprintf(stderr, "open failed: %s\n", mfsk_last_error());
        return 1;
    }

    MfskDecode rows[16];
    memset(rows, 0, sizeof rows);
    for (auto& r : rows) r.size = sizeof r;   // the caller's half of the growth contract
    size_t n = 0;
    if (mfsk_decoder_decode_i16(d, slot.data(), slot.size(), 12000, MFSK_PERIOD_NONE,
                                rows, 16, &n) != MFSK_STATUS_OK) {
        fprintf(stderr, "decode failed: %s\n", mfsk_decoder_last_error(d));
        mfsk_decoder_close(d);
        return 1;
    }
    for (size_t i = 0; i < n; ++i) {
        printf("%+7.1f Hz  dt=%+.2f s  SNR=%+.0f dB  %s\n",
               rows[i].freq_hz, rows[i].dt_sec, rows[i].snr_db, rows[i].text);
    }

    mfsk_decoder_close(d);   // the only thing with a lifetime
    return 0;
}
```

The runnable version lives at `examples/cpp_smoke/main.cpp` — every
mode's round trip, the capture stream, the decode callback, the budget,
the options a mode lacks being refused, and a multi-threaded stress. Build and run
with:

```
bash examples/cpp_smoke/build.sh
```
## ABI surface at a glance

Grouped as the header groups it. `mfsk.h`'s doc comments are the
authority on each function's contract; this table is a map, not a
specification.

**Introspection — ask the build what it has**

| Function | Role |
|---|---|
| `mfsk_abi_version` | The boundary's own revision (3). Moves when the C surface changes shape, unlike `mfsk_version`. |
| `mfsk_mode_count` / `mfsk_mode_at` | Enumerate the modes **this build** supports; protocols are feature-gated. |
| `mfsk_mode_name` / `mfsk_mode_from_name` | Round-trip a mode through its display name (`"FT8"`, `"FST4-120"`). |
| `mfsk_mode_info` | Geometry: tones, symbols, slot samples, TX offset, slot-FFT size, capability word. Size-versioned. |
| `mfsk_mode_caps` | Just the capability word — the `MFSK_CAP_*` bits. |

**Decoder — one handle for every slot mode** (FT8, FT4, FST4 ×5, WSPR, JT9, JT65, Q65 ×10)

| Function | Role |
|---|---|
| `mfsk_params_init` | Fill `MfskParams` — WSJT-X's per-period parameter block (`jt9com.f90`: depth, AP mode, band, Rx/Tx frequency, station and QSO) — with a mode's defaults. Zeroing it by hand is not equivalent. |
| `mfsk_extras_init` | Fill `MfskExtras` — the library's own options (strategy, sniper, AP hint, noise blanker, Q65 Pileup / Max Drift / fading, …) — with "unset". A zeroed one is not "unset". |
| `mfsk_decoder_open` / `mfsk_decoder_close` | The handle: one persistent decoder per mode, as `jt9 -s` is. Owns the callsign hash table, FT8's a7 list, Q65 and JT65 averages and WSPR's call table. An option the mode lacks is refused here with `MFSK_STATUS_UNSUPPORTED`. |
| `mfsk_decoder_set_params` / `_set_extras` | Change the blocks between periods; state is kept. |
| `mfsk_decoder_decode_i16` / `_f32` | Decode one period into caller-owned rows. `period` is the UTC-grid index, or `MFSK_PERIOD_NONE`. |
| `mfsk_decoder_decode_stream` | Decode the capture stream's slot in place, without copying it out and back. |
| `mfsk_decoder_set_on_decode` | Deliver rows as they are found, on top of the array. |
| `mfsk_decoder_set_budget` / `mfsk_decoder_last_budget` | A caller-supplied predicate polled during the search, and what it cut short (`MFSK_CAP_BUDGET`). |
| `mfsk_decoder_add_callsign` | Teach the table a call, so a later `<...>` resolves. |
| `mfsk_decoder_clear` | Forget what is carried between periods (WSJT-X's "Clear Avg"). |
| `mfsk_decoder_copy_info` | The FEC information bits of a row from the last decode. |
| `mfsk_decoder_set_q65_callers` | Q65: the contest callers list for the full-AP decode. |
| `mfsk_decoder_unpack77` | Render a packed message, resolving `<...>` through this decoder's table. |
| `mfsk_decoder_last_error` | This handle's error slot — survives a thread hop, unlike the global. |

**Streaming capture**

| Function | Role |
|---|---|
| `mfsk_stream_open` / `_close` | A holder for exactly one slot, sized from the mode; it cuts slots on the mode's UTC grid from the sample count. |
| `mfsk_stream_push_i16` / `_push_f32` | Feed it whatever the audio callback delivers; resampled if needed. |
| `mfsk_stream_position` / `mfsk_stream_set_time` | Say that sample `n` was at UTC `t`, as often as a reading arrives; the stream follows it at up to 400 ppm. The library reads no clock. |
| `mfsk_stream_slot_ready` / `_dropped` / `_take_slot_i16` / `_clear` | Poll, count slots replaced before they were taken, take, discard. |

**Transmit**

| Function | Role |
|---|---|
| `mfsk_pack77` / `_type1` / `_free_text` / `_type4` | Pack a message to 77 bits, one per byte. |
| `mfsk_unpack77` | Render packed bits as text, `<...>` left unresolved (`mfsk_decoder_unpack77` resolves them). |
| `mfsk_symbol_count` / `mfsk_synth_output_len` | The two buffer sizes the next stage needs. 0 means the mode has no tone stage. |
| `mfsk_message_to_tones` | Stage 2: packed message → channel symbols. |
| `mfsk_tones_to_i16` / `_f32` | Stage 3: symbols → 12 kHz PCM, into your buffer. |
| `mfsk_encode_ft8` / `_ft4` / `_fst4s60` / `_wspr` / `_jt9` / `_jt65` / `_q65` | One-call synthesis of a standard message, for callers that do not want the stages. |
| `mfsk_encode_q65_flagged` | `mfsk_encode_q65` with WSJT-X 3.2's Pileup "copied last Tx" flag set on the frame. |

**Q65 lists** (the decoder itself needs nothing beyond the handle: Pileup, Max Drift and fading are `MfskExtras`; the EME delay and averaging are `MfskParams::flags`; a Pileup reply's rows carry `MFSK_DECODE_FLAG_COPIED_LAST_TX`, `flags` bit 1)

| Function | Role |
|---|---|
| `mfsk_q65_history_new` / `_free` / `_push` / `_record` / `_lookup` / `_len` | `MfskQ65History`: the 100 most recent decodes (`q65_hist`); `_lookup` names the DX station near an Rx frequency. |
| `mfsk_q65_callers_new` / `_free` / `_record` / `_expire` / `_remove` / `_len` / `_get` | `MfskQ65Callers`: the contest list of up to 50 callers with grids (`q65_hist2`); times are the caller's Unix seconds. |

**Wideband IQ receiver** (`mfsk_iq_*`; one IQ stream, N channels, each with a decoder of its own)

| Function | Role |
|---|---|
| `mfsk_iq_open` / `_open_with` / `_close` | The receiver handle; `_open_with` picks the shared polyphase channelizer. |
| `mfsk_iq_add_channel` / `_remove_channel` | A dial frequency and a mode, with the `MfskParams` / `MfskExtras` of its decoder (NULL for defaults). |
| `mfsk_iq_channel_decoder` / `_channel_state` | The channel's borrowed `MfskDecoder*` (reconfigure it, never close or decode with it); active or paused. |
| `mfsk_iq_set_time` / `_retune` / `_gap` | Clock readings (400 ppm slew), a moved tuner (channels that no longer fit pause), lost samples. |
| `mfsk_iq_push` / `_poll` / `_pending` / `_samples_in` | Feed IQ bytes (decoding runs inside the push); drain `MfskIqDecode` rows. |

**JTTY receiver** (`MFSK_MODE_JTTY`, `MFSK_CAP_STREAM_RECEIVER`; needs the `jtty` feature, otherwise `MFSK_STATUS_UNKNOWN_PROTOCOL`)

| Function | Role |
|---|---|
| `mfsk_jtty_params_init` | Fill `MfskJttyParams` with `rjtty`'s defaults (1500 Hz ± 50 Hz, `smin` 4.6 dB, band 200–2800 Hz, subtraction on). Size-versioned. |
| `mfsk_jtty_open` / `_close` / `_set_params` | The receiver handle at any input sample rate (resampled to 12 kHz); settings apply from the next window. |
| `mfsk_jtty_push_i16` / `_push_f32` / `_finish` / `_reset` | Feed audio in any chunk size (decoding runs inside the push); `_finish` flushes messages still open at the end of a recording; `_reset` starts again at sample 0. |
| `mfsk_jtty_pending` / `mfsk_jtty_poll` | The queue of `MfskJttyUpdate`s, coalesced per message between polls; `poll` returns 1 per update written, 0 when none is waiting. |
| `mfsk_jtty_encode_tones` / `mfsk_jtty_synth_len` / `mfsk_jtty_tones_to_i16` / `_to_f32` | Transmit: text → tones (upstream's `pack_jtty` + `genjtty`, 59 tones per frame) → 12 kHz PCM. |

**Process-wide**

| Function | Role |
|---|---|
| `mfsk_runtime_configure` / `mfsk_runtime_thread_count` | Thread count, stack size and the two worker hooks. Call once, before the first decode. |
| `mfsk_last_error` | Thread-local last-error string. Prefer the decoder's where there is one. |
| `mfsk_version` | Library version, `major << 16 \| minor << 8 \| patch`. |

## Memory ownership

**Nothing crosses the boundary owned.** Decodes go into rows the caller
allocated, synthesis writes into buffers the caller sized from
`mfsk_symbol_count` / `mfsk_synth_output_len`, and the pack/unpack
family works in fixed-size arrays. There is no `*_free` for data,
because there is no data to free — which deletes the whole category of
leak that appears when an exception unwinds between a call and its
release, the thing that made the pre-v2 surface awkward from Kotlin and
Swift alike.

Six things do have lifetimes, and each has exactly one destructor:

- `MfskDecoder*` from `mfsk_decoder_open` → `mfsk_decoder_close`.
- `MfskStream*` from `mfsk_stream_open` → `mfsk_stream_close`.
- `MfskJttyReceiver*` from `mfsk_jtty_open` → `mfsk_jtty_close`.
- `MfskIqReceiver*` from `mfsk_iq_open` → `mfsk_iq_close`.
- `MfskQ65History*` from `mfsk_q65_history_new` →
  `mfsk_q65_history_free`.
- `MfskQ65Callers*` from `mfsk_q65_callers_new` →
  `mfsk_q65_callers_free`.

Three borrowed pointers, valid only for a bounded window: the row a
decode callback receives (that call only — copy what you keep), the
strings from `mfsk_last_error` / `mfsk_decoder_last_error` (until the
next fallible call on that thread, or on that handle), and a channel's
decoder from `mfsk_iq_channel_decoder` (until the channel is removed; never
closed by the caller).

## Thread safety

**One decoder per thread.** A decoder owns a callsign hash table and the
averages it mutates on every decode, so it is not safe to decode on one
from two threads. Concurrent decodes on *separate* decoders are supported
and exercised by `cpp_smoke` on every CI run (8 threads with their own
decoders, and a mixed-mode fan-out).

`mfsk_last_error` is thread-local, which is exactly the trap
`mfsk_decoder_last_error` exists for: a coroutine or `async` caller that
hops threads between the status and the message reads NULL from the
global. Prefer the handle's whenever there is one.

A decode callback and a budget predicate can both be invoked from rayon
worker threads on a `desktop` build, possibly concurrently — see
`mfsk_decoder_set_on_decode`'s doc comment for the exact contract, which
is *stronger* on a `mobile` build with no rayon.

## Mode selection

A decoder is bound to one mode when it is opened; there is no
auto-detect that tries several against the same audio. **Do not hardcode
the list** — that is what the introspection family is for: enumerate
`mfsk_mode_count` / `mfsk_mode_at`, and ask `mfsk_mode_info` for the
geometry.

```c
for (uint32_t i = 0; i < mfsk_mode_count(); ++i) {
    MfskMode m;
    mfsk_mode_at(i, &m);
    MfskModeInfo info = { .size = sizeof(MfskModeInfo) };
    mfsk_mode_info(m, &info);
    printf("%-10s %6.1f s  %8u samples  caps=%#llx\n",
           info.name, info.t_slot_s, info.slot_samples_12k,
           (unsigned long long)info.caps);
}
```

`MfskMode` addresses all 26 modes — every registry entry plus MSK144 and JTTY —
with discriminants that are ABI and never reordered. They are
deliberately **not** registry indices: membership is feature-gated, so a
build without `q65` would shift every index after it.

Every slot mode opens a decoder through the same `mfsk_decoder_*` handle —
FT8, FT4, the five FST4 sub-modes, WSPR, JT9, JT65 and the ten Q65 sub-modes —
and an option a mode lacks is refused at open with `MFSK_STATUS_UNSUPPORTED`
rather than dropped. `MFSK_CAP_DECODE_HANDLE` no longer says whether a decoder
opens; it marks the 77-bit-message family (FT8, FT4, FST4), the one the
QSO-context AP, a7 and the sniper window apply to. MSK144, JTTY and uvpacket have
no decoder (`MFSK_STATUS_UNKNOWN_PROTOCOL`); JTTY has `mfsk_jtty_*`.

Non-12 kHz input is resampled internally (linear interpolation). Sample
rates from 8 000 Hz up to at least 96 000 Hz work.
## License

GPL-3.0-only, matching [`mfsk-core`](https://github.com/jl1nie/mfsk-core)
and [WSJT-X](https://sourceforge.net/projects/wsjt/) upstream.
