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
> It exposes the FT8 decode + transmit slice with the fixed-point
> hot path enabled, distributed as prebuilt static libraries on the
> same [GitHub Releases](https://github.com/jl1nie/mfsk-core/releases) page
> (linux-x86_64 / esp32-xtensa / esp32s3-xtensa). `mfsk-ffi` (this
> crate) is the desktop/mobile-focused superset covering all seven
> WSJT modes — same footprint tradeoff either way, since desktop and
> mobile apps don't need the embedded crate's no_std/fixed-point
> constraints.

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
this header — `import MfskCore`, then `DecodeSession(mode: .ft8)`. For a
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

    // 3. Decode it. `params` may be NULL for the mode's published
    //    defaults, or a MfskDecodeParams you filled from
    //    mfsk_decode_params_init and adjusted.
    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskDecodeSession* s = mfsk_session_open(MFSK_MODE_FT8, nullptr, &st);
    if (s == nullptr) {
        fprintf(stderr, "open failed: %s\n", mfsk_last_error());
        return 1;
    }

    MfskDecode rows[16];
    memset(rows, 0, sizeof rows);
    for (auto& r : rows) r.size = sizeof r;   // the caller's half of the growth contract
    size_t n = 0;
    if (mfsk_session_decode_i16(s, slot.data(), slot.size(), 12000, nullptr,
                                rows, 16, &n) != MFSK_STATUS_OK) {
        fprintf(stderr, "decode failed: %s\n", mfsk_session_last_error(s));
        mfsk_session_close(s);
        return 1;
    }
    for (size_t i = 0; i < n; ++i) {
        printf("%+7.1f Hz  dt=%+.2f s  SNR=%+.0f dB  %s\n",
               rows[i].freq_hz, rows[i].dt_sec, rows[i].snr_db, rows[i].text);
    }

    mfsk_session_close(s);   // the only thing with a lifetime
    return 0;
}
```

The runnable version lives at `examples/cpp_smoke/main.cpp` — every
mode's round trip, the capture ring, the decode callback, the budget /
known / FFT-cache strategies, and a multi-threaded stress. Build and run
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
| `mfsk_abi_version` | The boundary's own revision. Moves when the C surface changes shape, unlike `mfsk_version`. |
| `mfsk_mode_count` / `mfsk_mode_at` | Enumerate the modes **this build** supports; protocols are feature-gated. |
| `mfsk_mode_name` / `mfsk_mode_from_name` | Round-trip a mode through its display name (`"FT8"`, `"FST4-120"`). |
| `mfsk_mode_info` | Geometry: tones, symbols, slot samples, TX offset, slot-FFT size, capability word. Size-versioned. |
| `mfsk_mode_caps` | Just the capability word — the `MFSK_CAP_*` bits. |
| `mfsk_mode_defaults` | That mode's published default search, with the scale its `sync_min` is measured on. |

**Decode session — every mode with `MFSK_CAP_DECODE_HANDLE`**

| Function | Role |
|---|---|
| `mfsk_decode_params_init` | Fill `MfskDecodeParams` with a mode's defaults. Zeroing it by hand is not equivalent. |
| `mfsk_session_open` / `mfsk_session_close` | The handle. Owns the callsign hash table and the previous slot's rows. |
| `mfsk_session_decode_i16` / `_f32` | Decode one slot into caller-owned rows. |
| `mfsk_session_decode_stream` | Decode the capture ring's slot in place, without copying it out and back. |
| `mfsk_session_set_on_decode` | Deliver rows as they are found, on top of the array. |
| `mfsk_session_set_budget` / `mfsk_session_last_budget` | A caller-supplied predicate polled during the search, and what it cut short (`MFSK_CAP_BUDGET`). |
| `mfsk_session_keep_known` / `mfsk_session_known_count` | Carry a decode's results into the next as known signals — skipped, or subtracted (`MFSK_CAP_KNOWN_FILTER` / `_SUBTRACT`). |
| `mfsk_session_keep_fft_cache` | Reuse the slot transform for a second pass over the same audio (`MFSK_CAP_FFT_CACHE`). Reuse is fingerprint-checked. |
| `mfsk_session_add_callsign` | Teach the table a call, so a later `<...>` resolves. |
| `mfsk_session_copy_info` | The FEC information bits of a row from the last decode. |
| `mfsk_session_last_error` | This handle's error slot — survives a thread hop, unlike the global. |

**Streaming capture**

| Function | Role |
|---|---|
| `mfsk_stream_open` / `_close` | A ring holding exactly one slot, sized from the mode. |
| `mfsk_stream_push_i16` / `_push_f32` | Feed it whatever the audio callback delivers; resampled if needed. |
| `mfsk_stream_set_epoch` | Say what UTC second the next sample belongs to. The library reads no clock. |
| `mfsk_stream_slot_ready` / `_buffered` / `_take_slot_i16` / `_clear` | Poll, measure, take, discard. |

**Transmit**

| Function | Role |
|---|---|
| `mfsk_pack77` / `_type1` / `_free_text` / `_type4` | Pack a message to 77 bits, one per byte. |
| `mfsk_unpack77` | Render packed bits as text, optionally resolving `<...>` through a session. |
| `mfsk_symbol_count` / `mfsk_synth_output_len` | The two buffer sizes the next stage needs. 0 means the mode has no tone stage. |
| `mfsk_message_to_tones` | Stage 2: packed message → channel symbols. |
| `mfsk_tones_to_i16` / `_f32` | Stage 3: symbols → 12 kHz PCM, into your buffer. |
| `mfsk_encode_ft8` / `_ft4` / `_fst4s60` / `_wspr` / `_jt9` / `_jt65` / `_q65` | One-call synthesis of a standard message, for callers that do not want the stages. |

**Modes with their own entry points** (no `MFSK_CAP_DECODE_HANDLE`)

| Function | Role |
|---|---|
| `mfsk_wspr_decode` | Scan a 120 s WSPR slot. |
| `mfsk_jt9_decode_at` / `mfsk_jt65_decode_at` | Point decode at a known carrier. |
| `mfsk_q65_decode` | Q65 plain AWGN scan — the baseline the other three trade against. |
| `mfsk_q65_decode_with_ap` | Up to four a-priori hints (~2 dB). |
| `mfsk_q65_decode_fading` | Fast-fading metric for microwave EME (5-8 dB on spread channels). |
| `mfsk_q65_decode_with_ap_list` | Template matching from a known call pair (~3 dB). |
| `mfsk_callsign_hash_table_new` / `_insert` / `_free` | The table Q65's family takes; a session owns its own. |

**Process-wide**

| Function | Role |
|---|---|
| `mfsk_runtime_configure` / `mfsk_runtime_thread_count` | Thread count, stack size and the two worker hooks. Call once, before the first decode. |
| `mfsk_last_error` | Thread-local last-error string. Prefer the session's where there is one. |
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

Three things do have lifetimes, and each has exactly one destructor:

- `MfskDecodeSession*` from `mfsk_session_open` → `mfsk_session_close`.
- `MfskStream*` from `mfsk_stream_open` → `mfsk_stream_close`.
- `MfskCallsignHashTable*` from `mfsk_callsign_hash_table_new` →
  `mfsk_callsign_hash_table_free`.

Two borrowed pointers, valid only for a bounded window: the row a
decode callback receives (that call only — copy what you keep), and the
strings from `mfsk_last_error` / `mfsk_session_last_error` (until the
next fallible call on that thread, or on that handle).

## Thread safety

**One session per thread.** A session owns a callsign hash table it
mutates on every decode and the previous slot's rows, so it is not safe
to decode on one from two threads. Concurrent decodes on *separate*
sessions are supported and exercised by `cpp_smoke` on every CI run (8
threads with their own sessions, and a mixed-mode fan-out).

`mfsk_last_error` is thread-local, which is exactly the trap
`mfsk_session_last_error` exists for: a coroutine or `async` caller that
hops threads between the status and the message reads NULL from the
global. Prefer the handle's whenever there is one.

A decode callback and a budget predicate can both be invoked from rayon
worker threads on a `desktop` build, possibly concurrently — see
`mfsk_session_set_on_decode`'s doc comment for the exact contract, which
is *stronger* on a `mobile` build with no rayon.

## Mode selection

A session is bound to one mode when it is opened; there is no
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

`MfskMode` addresses all 25 modes — every registry entry plus MSK144 —
with discriminants that are ABI and never reordered. They are
deliberately **not** registry indices: membership is feature-gated, so a
build without `q65` would shift every index after it.

Which decode entry point a mode takes is `MFSK_CAP_DECODE_HANDLE`:
FT8, FT4 and all five FST4 sub-modes drive `mfsk_session_*`; WSPR, JT9,
JT65 and the ten Q65 sub-modes have their own functions, because their
addressing model differs (Q65 takes a sub-mode discriminant of its own,
JT9/JT65 take a known carrier, WSPR takes a whole slot and searches it).
That is a shape difference, not a hierarchy.

Non-12 kHz input is resampled internally (linear interpolation). Sample
rates from 8 000 Hz up to at least 96 000 Hz work.
## License

GPL-3.0-or-later, matching [`mfsk-core`](https://github.com/jl1nie/mfsk-core)
and [WSJT-X](https://sourceforge.net/projects/wsjt/) upstream.
