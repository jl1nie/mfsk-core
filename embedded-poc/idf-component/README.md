# mfsk-core + ESP-IDF (C/C++) integration template

How to drop an mfsk-core decoder into a non-Rust ESP-IDF project.

**This directory is documentation only.** It used to describe a
skeleton of real files — `CMakeLists.txt`, `main/`, `components/`,
`shim/` — built around the `mfsk-ffi-ft8` crate. That crate was retired
in 0.11.0 and the skeleton was never carried forward, so what follows is
the wiring written out rather than a tree you can `idf.py build`. The
three production boards under `embedded-poc/` are the working examples;
`embedded-poc/embedded-shared/src/esp_dsp_fft.rs` is the file to copy
from.

## Why a Rust shim is needed, whatever you link

On the embedded path `mfsk-core` takes its FFT backend through an
**`extern "Rust"` symbol**:

```rust
#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_make_default_fft_planner()
    -> Box<dyn mfsk_core::engine::fft::FftPlanner>
{
    Box::new(EspDspPlanner::new())
}
```

and, if you enable the i16 spectrogram path, a second one —
`mfsk_core_make_default_fft_planner_16`, returning
`Box<dyn FftPlanner16>`. Note the underscore before `16`; the linker
will not forgive a near miss.

**Pure C cannot define these.** `extern "Rust"` is a different ABI with
different name mangling, and `Box<dyn Trait>` is not a C type. So every
non-Rust integration links at least one small Rust staticlib. Missing
the symbol is a link-time error, not a runtime surprise.

(Before 0.8.0 there was a third symbol, `mfsk_core_dot_q15_i32`, for
the legacy BASIS per-symbol DFT. Issue #162 removed it once the
Goertzel fill path made it dead weight — new integrations don't
implement it. See
[`EMBEDDED.md`](../../docs/reference/EMBEDDED.md#per-symbol-dft-goertzel).)

## Two ways to link, and which to pick

Once you are writing Rust anyway, the C ABI stops paying for itself —
which is why `mfsk-ffi-ft8` was retired and why all three boards here
call `mfsk-core` directly.

**A. Shim exports your own `extern "C"` entry points** — recommended
for a single-protocol MCU build. The shim depends on `mfsk-core`,
satisfies the FFT symbol, and exposes exactly the functions your
`main.c` needs. Smallest binary, and you choose the signatures.

**B. Link `mfsk-ffi`** — the full C ABI over every protocol. Build it
as a staticlib for the Xtensa target and you get the session/stream API
described in [`BINDINGS.md`](../../docs/reference/BINDINGS.md). You
*still* need a Rust shim for the FFT symbol. Larger, and it pulls
machinery a one-protocol build won't use, but nothing has to be
hand-written.

## Layout

```text
your-app/                      # esp-idf project root
├── CMakeLists.txt
├── sdkconfig.defaults         # bigger main task stack + heap perf
├── main/
│   ├── CMakeLists.txt
│   └── main.c                 # your application
├── components/mfsk/
│   ├── CMakeLists.txt         # imports the prebuilt .a + header
│   ├── include/mfsk_shim.h    # your shim's C header (option A), or
│   │                          # a copy of mfsk-ffi/include/mfsk.h (B)
│   └── lib/libmfsk_shim.a     # from the Rust build below
└── shim/
    ├── Cargo.toml             # depends on mfsk-core (A) or mfsk-ffi (B)
    ├── .cargo/config.toml     # target + panic=abort
    └── src/
        ├── lib.rs             # extern "C" entry points + the FFT symbol
        └── esp_dsp_fft.rs     # copied from embedded-shared/
```

`shim/Cargo.toml` for option A:

```toml
[lib]
crate-type = ["staticlib"]

[dependencies]
mfsk-core = { path = "../../../mfsk-core", default-features = false,
              features = ["alloc", "ft8", "fft-extern", "fixed-point"] }
```

`fixed-point` implies `nstep-half` and is what the boards ship; see
[`EMBEDDED.md`](../../docs/reference/EMBEDDED.md#cargo-features-for-embedded-use)
for the rest of the feature choices.

## Build flow

### 1. Build the Rust shim, once per target or mfsk-core change

```sh
cd shim
source ~/export-esp.sh                      # Xtensa toolchain

RUSTFLAGS="-C panic=abort" \
cargo build --release --target xtensa-esp32s3-espidf   # or xtensa-esp32-espidf
# → target/xtensa-esp32s3-espidf/release/libmfsk_shim.a

cp target/xtensa-esp32s3-espidf/release/libmfsk_shim.a \
   ../components/mfsk/lib/
```

`-C panic=abort` is required — Rust unwinding panics need `std`.
ESP-IDF projects usually put it in `.cargo/config.toml` instead:

```toml
[target.xtensa-esp32s3-espidf]
rustflags = ["-C", "link-arg=-nostartfiles", "-C", "panic=abort"]
```

For option B, copy `mfsk-ffi/include/mfsk.h` into
`components/mfsk/include/` as well — it is cbindgen-generated and
committed, so it needs no build step of its own.

### 2. Import the archive as a component

`components/mfsk/CMakeLists.txt`:

```cmake
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES espressif__esp-dsp)
add_library(mfsk_rust STATIC IMPORTED)
set_target_properties(mfsk_rust PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/libmfsk_shim.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE mfsk_rust)
```

`REQUIRES espressif__esp-dsp` is what makes the ASM FFT kernels
(`dsps_fft2r_fc32_ae32`, `dsps_fft2r_sc16_ae32`) available to the shim.

### 3. Build the ESP-IDF project

```sh
idf.py set-target esp32s3        # or esp32
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

For capturing a session log, prefer
`embedded-poc/scripts/flash-monitor.sh` — see `embedded-poc/CLAUDE.md`
for the two foot-guns it avoids.

## What this template does NOT cover

- Audio I/O — I2S / USB Audio capture and the 12 kHz ring are yours.
  `engine::dsp::resample` converts sample rates;
  `embedded-poc/embedded-shared/src/pipeline.rs` is the worked example.
- Time / NTP / GPS sync — slot alignment is the caller's job.
- Display, networking, OTA — out of scope.

These are the same explicit non-goals as
[`EMBEDDED.md`](../../docs/reference/EMBEDDED.md#what-we-dont-ship):
mfsk-core ships the decoder, this page ships the wiring, the
application is your code.

## Other targets

For RP2040 / RP2350-Hazard3 / Cortex-M, replace the copied
`esp_dsp_fft.rs` with an FFT bridge to your DSP library (CMSIS-DSP,
`microfft`, …) and change the target in `.cargo/config.toml`. Nothing
else about the wiring differs. Those targets are build-verified at best
— see
[`EMBEDDED.md`](../../docs/reference/EMBEDDED.md#other-targets--whats-verified-vs-aspirational)
for what has and has not actually been run.
