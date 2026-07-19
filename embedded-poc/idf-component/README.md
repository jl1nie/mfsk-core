# mfsk-ffi-ft8 + ESP-IDF (C/C++) integration template

A worked example showing how to drop FT8 decoding into a non-Rust
ESP-IDF project, via the [`mfsk-ffi-ft8`](../../mfsk-ffi-ft8/) crate.
The skeleton in this directory is intentionally minimal — it shows
the wiring, not a finished application.

```
idf-component/
├── README.md              ← this file
├── CMakeLists.txt         ← esp-idf project root
├── sdkconfig.defaults     ← bigger main task stack + heap perf
├── main/
│   ├── CMakeLists.txt
│   └── main.c             ← FT8 decode demo
├── components/
│   └── mfsk_ft8/
│       ├── CMakeLists.txt ← imports the prebuilt .a + header
│       ├── include/       ← (gitignored — populated by build step 1)
│       └── lib/           ← (gitignored — populated by build step 2)
└── shim/                  ← tiny Rust crate that bridges esp-dsp
    ├── Cargo.toml
    ├── .cargo/
    │   └── config.toml    ← target = xtensa-esp32-espidf, panic=abort
    └── src/
        ├── lib.rs         ← exports `mfsk_core_make_default_fft_planner`
        │                    (extern Rust)
        └── esp_dsp_fft.rs ← esp-dsp ASM bridges (vendored copy of
                              embedded-shared/src/esp_dsp_fft.rs)
```

## Why a Rust shim is needed

`mfsk-ffi-ft8` calls into `mfsk-core` which on the embedded path
takes its FFT backend through **one `extern "Rust"` symbol**:

- `mfsk_core_make_default_fft_planner()` — returns a boxed
  `Box<dyn FftPlanner>` for the protocol's FFT calls.

(Prior to 0.8.0 there was a second symbol,
`mfsk_core_dot_q15_i32`, for the legacy BASIS per-symbol DFT path —
removed in issue #162 once the Goertzel fill path made it dead
weight. New integrations don't need to implement it.)

Pure-C code can't define `extern "Rust"` symbols (different name
mangling, ABI assumptions). So we wrap the ESP-IDF `esp-dsp`
component in a tiny Rust shim crate (`shim/`) that:

1. Depends on `mfsk-ffi-ft8` (which carries the FT8 decoder).
2. Implements the extern Rust symbol by calling esp-dsp's
   `dsps_fft2r_*`.
3. Compiles to a `staticlib` (`libft8_shim.a`) that the ESP-IDF
   `mfsk_ft8` component imports.

End result: one `.a` to link, with the FFT/dot-product backend baked
in. The C app just calls `mfsk_ft8_decode_i16(...)` and gets results.

## Build flow

### 1. Build the Rust shim once per target / mfsk-core change

```sh
cd embedded-poc/idf-component/shim

# Source the Xtensa toolchain (if not already done)
source ~/export-esp.sh

# Build for ESP32 (Core2 / classic LX6)
cargo build --release --target xtensa-esp32-espidf
# → target/xtensa-esp32-espidf/release/libft8_shim.a

# Copy artifacts into the ESP-IDF component layout
cp target/xtensa-esp32-espidf/release/libft8_shim.a \
   ../components/mfsk_ft8/lib/
cp ../../../mfsk-ffi-ft8/include/mfsk_ft8.h \
   ../components/mfsk_ft8/include/
```

### 2. Build the ESP-IDF project

```sh
cd embedded-poc/idf-component
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## What this template does NOT do

- Audio I/O — `main.c` decodes a baked-in WAV. Wire your I2S /
  microphone in your fork.
- Time / NTP / GPS sync — slot alignment is the caller's job.
- Display, networking, OTA — out of scope.

These are the same explicit non-goals as
[`docs/reference/EMBEDDED.md`](../../docs/reference/EMBEDDED.md): mfsk-core ships the
decoder, the integration template ships the wiring, the application
is your code.

## Other targets

For RP2040 / RP2350-Hazard3 / Cortex-M, replace `shim/`'s
`esp_dsp_fft.rs` with an FFT bridge to your DSP library
(CMSIS-DSP / arm-dsp, etc.) and adjust `shim/.cargo/config.toml`'s
target. The mfsk-ffi-ft8 / ESP-IDF component wiring is the same.
