# mfsk-ffi

C ABI wrapper around [`mfsk-core`](https://crates.io/crates/mfsk-core) that
exposes the WSJT-family decoders and synthesisers to C, C++, and JNI
(Android) consumers. Not published to crates.io — consumers clone this
repo and link against the local `cdylib` / `staticlib`.

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

See `examples/kotlin_jni/` for a JNI skeleton.

## Quick start (C++)

The complete end-to-end flow is encode → decode through the ABI:

```cpp
#include "mfsk.h"
#include <cstdio>
#include <cstring>

int main() {
    // 1. Synthesise an FT8 message at 1500 Hz.
    MfskSamples wave = {0};
    if (mfsk_encode_ft8("CQ", "JA1ABC", "PM95", 1500.0f, &wave) != MFSK_STATUS_OK) {
        fprintf(stderr, "encode failed: %s\n", mfsk_last_error());
        return 1;
    }

    // 2. Decode the synthesised audio.
    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_FT8);
    MfskMessageList list = {0};
    MfskStatus st = mfsk_decode_f32(dec, wave.samples, wave.len, 12000, &list);
    if (st != MFSK_STATUS_OK) {
        fprintf(stderr, "decode failed: %s\n", mfsk_last_error());
        return 1;
    }

    // 3. Print results.
    for (size_t i = 0; i < list.len; ++i) {
        printf("%+7.1f Hz  dt=%+.2f s  SNR=%+.0f dB  %s\n",
               list.items[i].freq_hz,
               list.items[i].dt_sec,
               list.items[i].snr_db,
               list.items[i].text);
    }

    // 4. Release everything.
    mfsk_message_list_free(&list);
    mfsk_samples_free(&wave);
    mfsk_decoder_free(dec);
    return 0;
}
```

The runnable version lives at `examples/cpp_smoke/main.cpp`; it exercises
all six protocols + multi-threaded decode stress tests. Build and run
with:

```
bash examples/cpp_smoke/build.sh
```

## ABI surface at a glance

| Function                   | Role                                                              |
|----------------------------|-------------------------------------------------------------------|
| `mfsk_decoder_new`         | Construct opaque decoder handle for one protocol.                 |
| `mfsk_decoder_free`        | Destroy decoder handle.                                           |
| `mfsk_decode_f32`          | Decode one slot of `f32` PCM.                                     |
| `mfsk_decode_i16`          | Decode one slot of `i16` PCM (same semantics as `_f32`).          |
| `mfsk_message_list_free`   | Release the list returned by a decode.                            |
| `mfsk_encode_ft8`          | Synthesise a standard FT8 message (`call1 call2 report`).         |
| `mfsk_encode_ft4`          | Synthesise a standard FT4 message.                                |
| `mfsk_encode_fst4s60`      | Synthesise an FST4-60A message.                                   |
| `mfsk_encode_wspr`         | Synthesise a Type-1 WSPR message (`call grid power_dbm`).         |
| `mfsk_encode_jt9`          | Synthesise a standard JT9 message.                                |
| `mfsk_encode_jt65`         | Synthesise a standard JT65 message.                               |
| `mfsk_samples_free`        | Release the `f32` buffer returned by an encode.                   |
| `mfsk_last_error`          | Thread-local last-error string (UTF-8, NUL-terminated).           |
| `mfsk_version`             | Library version (major << 16 \| minor << 8 \| patch).             |

## Memory ownership

- **Decode**: caller allocates a zero-initialised `MfskMessageList`;
  the decoder fills `items` / `len`. The caller must release the list
  with `mfsk_message_list_free`, which frees each message's `text`
  pointer and the `items` array itself.
- **Encode**: caller allocates a zero-initialised `MfskSamples`; the
  encoder fills `samples` / `len`. The caller must release with
  `mfsk_samples_free`.
- **Error strings** (`mfsk_last_error`) are thread-local and remain
  valid until the next fallible call on the same thread.

Never mix allocators: do not `free()` a pointer that came from the
library, and do not pass caller-allocated arrays to the `*_free`
functions. If you zero-initialise then never call the corresponding
decode / encode (e.g. because an earlier call errored), the `*_free`
functions are still safe — they treat a `NULL` `items` / `samples`
pointer as a no-op.

## Thread safety

The supported model is **one decoder handle per thread**. In the
current implementation the handle carries no mutable state beyond its
protocol tag, so sharing one handle across threads also works — the
`cpp_smoke` driver explicitly exercises both patterns (8 threads ×
own handle, 8 threads × shared handle, mixed-protocol fan-out) on every
CI run. Future releases may add cached state to the handle; consult
this README for the then-current contract.

`mfsk_last_error` uses thread-local storage, so error text never
crosses threads.

## Protocol selection

Each `MfskDecoder` is bound to one protocol at construction time.
There is no runtime "auto-detect" that tries multiple protocols on the
same audio. Pick the protocol tag that matches the slot you are
decoding:

| Protocol tag                | Slot length | Sample-rate-agnostic input   |
|-----------------------------|-------------|------------------------------|
| `MFSK_PROTOCOL_FT8`         | 15 s        | 180 000 samples @ 12 kHz     |
| `MFSK_PROTOCOL_FT4`         | 7.5 s       | 90 000 samples @ 12 kHz      |
| `MFSK_PROTOCOL_FST4S60`     | 60 s        | 720 000 samples @ 12 kHz     |
| `MFSK_PROTOCOL_WSPR`        | 120 s       | 1 440 000 samples @ 12 kHz   |
| `MFSK_PROTOCOL_JT9`         | 60 s        | 720 000 samples @ 12 kHz     |
| `MFSK_PROTOCOL_JT65`        | 60 s        | 720 000 samples @ 12 kHz     |

Non-12 kHz input is resampled internally (linear interpolation).
Sample rates from 8 000 Hz up to at least 96 000 Hz work.

## License

GPL-3.0-or-later, matching [`mfsk-core`](https://github.com/jl1nie/mfsk-core)
and [WSJT-X](https://sourceforge.net/projects/wsjt/) upstream.
