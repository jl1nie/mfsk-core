# Embedded targets

`mfsk-core` is `no_std + alloc` capable: the FT8 decode path
(`mfsk_core::ft8::decode_block`) runs on chips with as little as
~150 KB of RAM when paired with caller-supplied FFT and dot-product
backends. This document covers what the library asks of an embedded
caller and what we don't ship.

## Architecture: how f32 and fixed-point share one codebase

The whole DSP / FEC pipeline is parameterised by **scalar traits**, so
the same source compiles to either a host-friendly f32 path or an
embedded-friendly integer path **with no duplicated code**:

- [`core::scalar::SpecScalar`] — spectrogram / DFT-output scalar
  (`f32` on host; `Q14i16` for embedded cs storage).
- [`core::scalar::LlrScalar`] — LLR scalar with wide-accumulator
  type (`f32` on host; `Q11i16` with i32 wide for embedded BP).
  As of 0.6.2 the embedded LLR scalar is `Q11i16` again (i16,
  1/2048 resolution); the `Q3i8` (i8, 1/8) variant evaluated
  during the 0.5.x line was rolled back when real-S3 testing on
  `qso3_busy.wav` showed +1 entry recall (XE2X HA2NP RR73) and a
  faster post-SlotEnd time (1.341 s → 1.191 s) under Q11i16
  despite the doubled scalar width.
- [`core::scalar::Cmplx<S>`] — generic complex over a `SpecScalar`,
  `repr(C)` layout-compatible with `num_complex::Complex`.
- `compute_llr_generic<P, S, T>`, `compute_snr_db_generic<P, S>`,
  `bp_decode_generic_nms<P, T>` — all take the scalar types as
  generic parameters; one monomorphisation per `(P, S, T)` triple.

The `fixed-point` Cargo
features just **swap which scalar types the protocol glue picks**;
the generic body is unchanged. This means the embedded port shares
99 % of its code with the host build — bug fixes and optimisations
land once and apply everywhere.

### What the fixed-point switch is wired up to today

| Component | Generic over | Fixed-point switch wired? |
|---|---|---|
| LDPC BP NMS (`fec::ldpc::bp`) | `LlrScalar` | ✅ via `fixed-point` |
| LLR computation (`core::llr`) | `SpecScalar` × `LlrScalar` | ✅ via `fixed-point` |
| BP scratch pool (`BpScratch<P, T>`) | `LdpcParams` × `LlrScalar` | ✅ — works for FT8 LDPC(174,91) and FST4/uvpacket LDPC(240,101) |
| FT8 spectrogram + DFT (`ft8::decode_block`) | `SpecScalar` × `AudioSample` | ✅ via `fixed-point` |
| **FT4 / WSPR / Q65 / JT9 / JT65** | (host f32 only) | ❌ — these protocols don't go through `decode_block` today |

So: **the trait infrastructure is protocol-agnostic, but the only
protocol that actually flips into the integer path on the embedded
build is FT8.** Adding FT4 (next-most-likely candidate, since it
shares the same Costas/Gray/LDPC pieces) is a port of the
`decode_block` shape to FT4-specific symbol layout — nothing new
in the trait layer.

## What we test

| Target | MCU | Backend | Status |
|---|---|---|---|
| **M5Stack Core2** | **ESP32-D0WD-V3** (Xtensa LX6, dual-core 240 MHz, single-issue f32 FPU, 16 MB flash, ~4 MB PSRAM) — confirmed by `espflash board-info`: `Chip type: esp32 (revision v3.1)` / `Features: WiFi, BT, Dual Core, 240MHz`. **Not** an ESP32-S2 (LX7, single-core, no BT) or S3. | esp-dsp ASM (`dsps_dotprod_s16_ae32`, `dsps_fft2r_*`) | Reference real-audio bench. See benchmark + footprint sections below. |
| ESP32-S3 (DevKitC) | Xtensa LX7 + PIE SIMD | esp-dsp ASM | Earlier reference; same `fft-extern` contract. |

### Other targets — what's verified vs aspirational

The `fft-extern` + `dotprod-extern` contracts are *designed* to be
target-portable, and `mfsk-ffi-ft8` cross-builds cleanly to several
non-Xtensa MCUs:

| Target | `cargo build` clean | FFT/dotprod shim shipped | Hardware-tested |
|---|---|---|---|
| `xtensa-esp32-espidf` | ✅ | ✅ esp-dsp (Core2) | ✅ qso1/2/3 sweep |
| `xtensa-esp32s3-espidf` | ✅ | ✅ esp-dsp (S3 PoC) | ✅ earlier reference |
| `thumbv8m.main-none-eabihf` (RP2350 Cortex-M33) | ✅ | ❌ candidates: CMSIS-DSP via pico-sdk-rs | ❌ |
| `riscv32imac-unknown-none-elf` (RP2350 Hazard3) | ✅ | ❌ no DSP library; `microfft` for FFT, scalar Rust for dot product | ❌ |
| `thumbv7em-none-eabihf` (Cortex-M4F / M7) | not tried | ❌ candidates: CMSIS-DSP `arm_*_q15` | ❌ |
| `thumbv6m-none-eabi` (Cortex-M0+ / RP2040) | not tried | ❌ scalar Rust only (no DSP unit) | ❌ |

**ESP32 / ESP32-S3** (Xtensa LX6 / LX7) are the only targets we
exercise end-to-end with real audio. For everything else, the
library *can* build (try
`cargo build -p mfsk-ffi-ft8 --release --no-default-features
--features embedded-fixed-point,embedded-runtime --target <T>`),
but you'll need to supply the two extern Rust symbols yourself.
Concrete RP2040 / RP2350 / Cortex-M shims are tracked as future
work.

`embedded-poc/m5stack-core2/` is the worked example to copy from.

## Cargo features for embedded use

Default features include `std`, `parallel`, and `fft-rustfft` — turn
those off and pick the embedded baseline:

```toml
[dependencies]
mfsk-core = { version = "0.6", default-features = false, features = [
    "alloc",            # Vec / Box / String — required for decode
    "ft8",              # FT8 protocol glue
    "fft-extern",       # caller supplies the FFT backend
    "fixed-point",      # u16 spec + i16 DFT + Q11i16 LLR + i16 NMS BP
    # Optional:
    # "profile-coarse",            # always-on stage-2 sub-stage timing
] }
```

Stage-3 sensitivity is now a runtime parameter on
`process_candidates_into` (`q_thresh: u32`), not a Cargo feature.
[`mfsk_core::ft8::decode_block::DEFAULT_Q_THRESH`] is 12 — full
recall on every target we currently ship for. We A/B-tested 12 vs 14
on both LX6 (Core2) and LX7 (M5StickS3) post-Phase-E + work-stealing
(`logs/core2_q_sweep_2026-05-04.log`, `logs/s3_q_sweep_2026-05-04.log`)
and found the relaxed q=14 saves only **0–78 ms on qso3 only** while
costing **one weak decode on qso3** on each chip (a different
borderline cand on each — W1DIG -15.5 dB on S3, N1PJT -18 dB on
Core2). At sub-1.5 s post-SlotEnd on Core2 / sub-0.8 s on S3, the
recall hit isn't worth the saving — keep `q_thresh` at the default.

Feature reference:

| Feature | What it changes | When to use |
|---|---|---|
| `std` | Pulls in `std::env`, `std::time::Instant`. Decoupled from rustfft. | esp-idf-svc-style targets that have std. Optional on bare-metal. |
| `alloc` | `extern crate alloc` + Vec / Box. | All decode paths. |
| `fft-extern` | FFT backend via `mfsk_core_make_default_fft_planner` extern fn. | Any embedded target. |
| `fft-rustfft` | rustfft as the FFT backend. | Host only. |
| `fixed-point` | Embedded integer pipeline: u16 spectrogram + i16 internal DFT + Q11i16 LLR + integer NMS BP. | Any embedded target — recall-close to the host f32 path with halved PSRAM bandwidth on the spec/DFT side. (The `Q3i8` LLR variant tested through 0.5.x was reverted in 0.6.2 after real-S3 testing showed Q11i16 added back +1 entry recall and ran faster despite the wider scalar — see §"Architecture".) |
| `profile-coarse` | Always emits coarse_sync sub-stage timings to stderr. | Diagnosis only. |

## The two extern Rust contracts

### FFT backend

`mfsk_core::core::fft::FftPlanner` is the decode path's FFT trait.
Under `fft-extern`, the library expects the binary to provide an
`extern "Rust"` factory:

```rust
#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_make_default_fft_planner()
    -> Box<dyn mfsk_core::core::fft::FftPlanner>
{
    Box::new(MyEspDspPlanner::new())
}
```

`embedded-poc/m5stack-core2/src/esp_dsp_fft.rs` is a working example
that bridges to esp-dsp's Xtensa ASM kernels (`dsps_fft2r_fc32_ae32`
+ `dsps_fft2r_sc16_ae32` for the i16 path). RP2040 / Cortex-M
implementations would bridge to CMSIS-DSP similarly.

### i16 × Q15 dot product

Under `fft-extern + fixed-point` the per-symbol DFT in
`ft8::decode_block` calls into `mfsk_core::core::dotprod::dot_q15_i32`,
which is a Rust scalar fallback by default but overridable via
another extern symbol:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "Rust" fn mfsk_core_dot_q15_i32(
    a: *const i16,
    b: *const i16,
    n: usize,
) -> i32 {
    // bridge to dsps_dotprod_s16_ae32 (LX6/LX7) or
    // arm_dot_prod_q15 (Cortex-M with CMSIS-DSP), etc.
}
```

This is the inner kernel of stage 3 — getting it onto the
target's native MAC unit is a big perf win on cached-RAM targets.
On LX6 with esp-dsp: roughly 1 cycle per 2 MACs.

## BASIS scratch

`fill_symbol_spectra_into` (and the wrappers `decode_block_into`,
`process_candidates_into`, `refine_candidates_into`) takes
caller-provided i16 scratch for the rotator basis:

```rust
const SCRATCH_LEN: usize = mfsk_core::ft8::decode_block::BASIS_SCRATCH_LEN;
static mut BASIS_RE: [i16; SCRATCH_LEN] = [0; SCRATCH_LEN];
static mut BASIS_IM: [i16; SCRATCH_LEN] = [0; SCRATCH_LEN];
```

`BASIS_SCRATCH_LEN = NTONES × NSPS = 15 360` (≈ 30 KB per axis,
60 KB total). This buffer is the **dot-product inner-loop hot
data** — the `dsps_dotprod_s16_ae32` ASM kernel runs at 1 cycle per
2 MACs only when basis sits in fast internal SRAM (DRAM). On a
Core2 with PSRAM enabled (the default), a PSRAM-resident basis
costs 5–10 cycles per term through the cache, dropping the kernel
to ~30 % of its rated speed and **doubling-to-tripling stage 3
wall-clock**. The static-array form lands in `.bss` automatically;
for heap allocation use ESP-IDF's capability-flagged allocator:

```c
int16_t *basis = heap_caps_malloc(BASIS_SCRATCH_LEN * sizeof(int16_t),
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
```

A plain `malloc` request of 60 KB silently lands in PSRAM under
ESP-IDF's default routing. Keep the buffers across decode calls;
they don't need to be reset between slots.

The `_into` family of `pub fn` exists specifically so embedded
callers can thread the scratch through without per-decode
allocation. The non-`_into` variants (`decode_block`, etc.)
heap-allocate at the default heap, which on PSRAM-enabled boards
is the slow path described above — fine for host but a footgun on
embedded.

## Q-format quick reference

| Stage | Format | Range | File |
|---|---|---|---|
| Spectrogram cell | u16 (mag²) | `>> FP_SPEC_SHIFT (12)` | `ft8::decode_block::Spectrogram` |
| DFT basis | Q15 i16 (cos, sin) | ±2¹⁵ ≈ ±1.0 | `fill_symbol_spectra_into` |
| Symbol cs | `Cmplx<f32>` (default) or `Cmplx<Q14i16>` (manual via `core::scalar`) | f32 unbounded; Q14 ±2 | `core::scalar::Cmplx` |
| LLR | f32 (host) or Q11i16 (`fixed-point`) | f32 unbounded; Q11 ±16, 1/2048 LSB | `core::scalar::LlrScalar` |
| BP messages | T (same as LLR) | — | `fec::ldpc::bp::bp_decode_generic_nms_with_scratch` |

## Using from C / C++ / non-Rust ESP-IDF projects (`mfsk-ffi-ft8`)

[`mfsk-ffi-ft8`](https://github.com/jl1nie/mfsk-core/tree/main/mfsk-ffi-ft8)
exposes a tiny C ABI for the FT8 block decoder slice. It is the
recommended way to call the embedded FT8 decoder from a non-Rust
ESP-IDF (or RP2040 / Cortex-M) project.

The crate is `no_std + alloc` under its `embedded-fixed-point`
feature so the resulting `libmfsk_ft8.a` doesn't carry Rust's `std`
runtime — drop-in linkable from C without the toolchain weirdness
that would come from mixing two libc layers.

**Verified end-to-end on ESP32 Core2** (m5stack-core2 example): a
separate `ffi_smoke_one` path calls `mfsk_ft8_decode_i16` (C ABI)
on the same baked WAVs as the direct-Rust `decode_one` path and
gets identical recall — qso1 (3 / 3), qso2 (5 / 5),
**qso3 busy band (7 / 7)**. With caller-managed BASIS scratch in
internal RAM the FFI path lands ~2.6 × faster than the same FFI
call with internal heap allocation (qso3 3.74 s vs 9.57 s) and
within ~5 % of the direct-Rust process_candidates_into path.
Logs:
- `embedded-poc/m5stack-core2/logs/ffi_into_2026-05-02.log`
  (recommended caller-scratch path)
- `embedded-poc/m5stack-core2/logs/ffi_smoke_2026-05-02.log`
  (heap-alloc reference for comparison)

### API at a glance

cbindgen-generated header — `mfsk-ffi-ft8/include/mfsk_ft8.h`,
regenerated on every build. The full surface is:

```c
typedef struct MfskFt8Result {
    char     text[40];   // NUL-terminated unpacked message
    float    freq_hz;    // carrier
    float    dt_sec;     // time offset relative to slot start
    float    snr_db;     // calibrated against JTDX absolute via
                         // xsnr2_db_simple (within ±3 dB on real silicon)
    uint32_t hard_errors;
    uint8_t  pass;       // staircase stage (0=fast Bp, 1=full Bp,…)
} MfskFt8Result;

typedef struct MfskFt8ResultList {
    MfskFt8Result *items;
    size_t         len;
    size_t         _capacity;  // private
} MfskFt8ResultList;

// Required scratch length for the primary decode entry, in i16
// elements. Caller allocates two arrays of this length each.
size_t mfsk_ft8_basis_scratch_len(void);

// PRIMARY embedded entry. Caller-managed scratch — must live in
// fast internal RAM (NOT PSRAM) for the dot-product ASM kernel to
// hit peak throughput.
MfskFt8Status mfsk_ft8_decode_i16(
    const int16_t *audio, size_t n_samples,   // 12 kHz, mono, ≥168 000
    float freq_min_hz, float freq_max_hz,     // typical 200, 3000
    float sync_min, int max_cand,             // typical 1.0, 30
    MfskFt8Depth depth,                       // 0=Bp, 1=BpAll, 2=BpAllOsd
    int16_t *basis_re, int16_t *basis_im,     // scratch
    MfskFt8ResultList *out);                  // populated by callee

// HOST-ONLY convenience — heap-allocs basis internally. Excluded
// from embedded builds: see "Why caller-supplied scratch" below.
#ifdef MFSK_FT8_HOST  // built with default `host` feature
MfskFt8Status mfsk_ft8_decode_i16_alloc(
    const int16_t *audio, size_t n_samples,
    float freq_min_hz, float freq_max_hz,
    float sync_min, int max_cand,
    MfskFt8Depth depth,
    MfskFt8ResultList *out);
#endif

void mfsk_ft8_result_list_free(MfskFt8ResultList *list);
```

### Caller-managed BASIS scratch

The C ABI mirrors the Rust `_into` family — `mfsk_ft8_decode_i16`
takes the basis arrays as parameters rather than hiding a `malloc`
inside. The reasoning is the same as documented in
[BASIS scratch](#basis-scratch) above (PSRAM placement breaks the
ASM kernel); for the FFI path the consequence is that the decoder
deliberately does **not** ship a "convenience" wrapper that
internally `malloc`s, because the C caller can't tell which heap
the allocation lands in. The caller decides:

```c
// The simplest correct pattern: static .bss arrays — they land
// in internal DRAM automatically and persist for the process
// lifetime.
#include "mfsk_ft8.h"
static int16_t basis_re[15360];   // = mfsk_ft8_basis_scratch_len()
static int16_t basis_im[15360];

MfskFt8ResultList results = {0};
MfskFt8Status st = mfsk_ft8_decode_i16(
    audio, n_samples,
    200.0f, 3000.0f, 1.0f, 30,
    MFSK_FT8_DEPTH_BP_ALL,
    basis_re, basis_im,
    &results);
```

### Streaming capture: I2S / USB Audio → 12 kHz ring

`mfsk_ft8_decode_i16` takes one 15-second 12 kHz slot at a time. Real
receivers don't have that — they get small DMA chunks at whatever
rate the codec runs (typically 16 / 24 / 48 kHz from I2S or USB
Audio Class 1/2). The `mfsk_ft8_stream_*` family bridges the two
sides without each consumer reinventing it:

```c
typedef struct MfskFt8Stream MfskFt8Stream;

// Construct: arbitrary src rate + ring capacity in 12 kHz samples.
// Pass 180000 for the standard 15 s slot.
MfskFt8Stream *mfsk_ft8_stream_new(uint32_t src_rate_hz, size_t cap);
void           mfsk_ft8_stream_free(MfskFt8Stream *);

// Push DMA chunk. Resamples to 12 kHz internally and appends to the
// ring (oldest samples overwritten when full — rolling-window model).
MfskFt8Status mfsk_ft8_stream_push_i16(MfskFt8Stream *,
                                       const int16_t *samples, size_t n);

// Snapshot: copy the most recent `cap` 12 kHz samples into `out`.
// Does not modify the ring — call _drain() after a successful decode
// to free room for new audio.
size_t mfsk_ft8_stream_buffered_samples(const MfskFt8Stream *);
size_t mfsk_ft8_stream_peek_latest(const MfskFt8Stream *,
                                   int16_t *out, size_t cap);
void   mfsk_ft8_stream_drain(MfskFt8Stream *, size_t n);
void   mfsk_ft8_stream_clear(MfskFt8Stream *);
```

Internals: a Q32 fixed-point linear resampler with carry-over state
(no chunk-boundary glitches) plus a fixed-cap i16 ring. Pure scalar
arithmetic — no FFT, no DSP backend. Available in both `host` and
`embedded-fixed-point` builds.

**Typical RTOS wiring** (capture and decode on different tasks):

```c
// One-time setup
static MfskFt8Stream *g_stream;
static int16_t g_slot[180000];          // 360 KB; OK in PSRAM
static int16_t g_basis_re[15360];       // MUST be in internal DRAM
static int16_t g_basis_im[15360];

void rx_init(void) {
    g_stream = mfsk_ft8_stream_new(/*src*/16000, /*cap*/180000);
}

// Capture task: I2S DMA callback
void on_i2s_chunk(const int16_t *samples, size_t n) {
    mfsk_ft8_stream_push_i16(g_stream, samples, n);
}

// Decode task: fires every 15 s on UTC slot boundary
void on_slot_boundary(void) {
    if (mfsk_ft8_stream_buffered_samples(g_stream) < 168000) return;
    mfsk_ft8_stream_peek_latest(g_stream, g_slot, 180000);

    MfskFt8ResultList results = {0};
    mfsk_ft8_decode_i16(g_slot, 180000,
                        200.0f, 3000.0f, 1.0f, 30,
                        MFSK_FT8_DEPTH_BP_ALL,
                        g_basis_re, g_basis_im, &results);
    // ... use results, then ...
    mfsk_ft8_result_list_free(&results);
    mfsk_ft8_stream_drain(g_stream, 180000);  // make room for next slot
}
```

**Slot-boundary alignment.** UTC alignment to within ±2 s is
sufficient — `decode_block`'s coarse-sync stage absorbs that much
drift internally via the Costas-array search. NTP is the easiest
source on Wi-Fi-enabled boards; GPS PPS works for offline / mobile
operation; for stand-alone benches, freerunning at exactly 15 s
intervals starting from any reference moment also decodes fine
provided the timer is stable to better than ~50 ppm over an hour.

**Resampler quality.** Linear interpolation — chosen for arithmetic
simplicity (i64 multiply / shift, fits comfortably on FPU-less MCUs
and tracks ASM throughput on LX6/LX7). For typical 16 → 12 kHz or
48 → 12 kHz ratios on real audio passbands (200–3000 Hz) the
introduced distortion is ~–55 dBc, well below the FT8 LDPC's
operating SNR. If you need transparent fidelity for downstream uses
beyond FT8, replace this stage with a polyphase FIR before the ring.

### Build flags

#### Host (`libmfsk_ft8.so` / `libmfsk_ft8.a` for desktop testing)

```sh
cargo build -p mfsk-ffi-ft8 --release
# → target/release/libmfsk_ft8.{so,a}
# → mfsk-ffi-ft8/include/mfsk_ft8.h (cbindgen-generated)
```

Default features pull `mfsk-core/std + ft8 + fft-rustfft`. A C smoke
test linking the resulting `.so` lives at
`mfsk-ffi-ft8/tests/c_smoke/smoke.c`:

```sh
gcc -O2 -I mfsk-ffi-ft8/include \
    mfsk-ffi-ft8/tests/c_smoke/smoke.c \
    -L target/release -lmfsk_ft8 -lm -lpthread -ldl \
    -Wl,-rpath,$PWD/target/release \
    -o /tmp/mfsk_smoke
/tmp/mfsk_smoke embedded-poc/m5stack-core2/assets/qso3_busy.wav
```

#### Embedded (Xtensa ESP32, `libmfsk_ft8.a` for ESP-IDF link)

```sh
source ~/export-esp.sh                     # Xtensa toolchain
RUSTFLAGS="-C panic=abort" \
cargo build -p mfsk-ffi-ft8 --release \
    --no-default-features \
    --features embedded-fixed-point,embedded-runtime \
    --target xtensa-esp32-espidf
# → target/xtensa-esp32-espidf/release/libmfsk_ft8.a
```

`-C panic=abort` is required because Rust unwinding panics need
`std`; embedded uses `panic = "abort"` everywhere. ESP-IDF projects
typically set this in their `.cargo/config.toml`:

```toml
[target.xtensa-esp32-espidf]
rustflags = ["-C", "link-arg=-nostartfiles", "-C", "panic=abort"]
```

#### Feature reference

| Feature | Default | Purpose |
|---|---|---|
| `host` | ✓ | Host build — pulls `mfsk-core/std + ft8 + fft-rustfft`. Both `mfsk_ft8_decode_i16` (caller scratch) and `mfsk_ft8_decode_i16_alloc` (heap convenience) are exported. |
| `embedded-fixed-point` | — | `no_std + alloc`. Pulls `mfsk-core/fft-extern + fixed-point`. **Only `mfsk_ft8_decode_i16` is exported** — the heap-alloc convenience is excluded by design (see above). The linker must resolve `mfsk_core_make_default_fft_planner_*` and `mfsk_core_dot_q15_i32` (typically via a small Rust shim that bridges esp-dsp). |
| `embedded-runtime` | — | Provides default `#[panic_handler]` (calls libc `abort`) + `#[global_allocator]` (libc `malloc`/`free`). Needed for a self-contained `staticlib`; turn off when stacking another Rust runtime in the same image. |

### Linking it into an ESP-IDF (CMake) project

```
your-app/                          # esp-idf project root
├── main/main.c                    # calls mfsk_ft8_decode_i16(...)
├── components/mfsk_ft8/
│   ├── CMakeLists.txt             # IMPORTED static-lib component
│   ├── include/mfsk_ft8.h         # from mfsk-ffi-ft8 build
│   └── lib/libmfsk_ft8.a          # from mfsk-ffi-ft8 build
└── shim/                          # tiny Rust crate (esp-dsp bridges)
    ├── Cargo.toml                 # depends on mfsk-ffi-ft8
    ├── .cargo/config.toml         # target = xtensa-esp32-espidf, panic=abort
    └── src/lib.rs                 # provides mfsk_core_make_default_fft_planner
                                   # and mfsk_core_dot_q15_i32 via esp-dsp
```

The `shim/` Rust crate is needed because mfsk-core's FFT-extern
contract uses `extern "Rust"` symbols (different ABI from `extern
"C"`), which a pure-C compilation unit can't satisfy. The shim is
~50 lines of Rust + a vendored copy of
`embedded-poc/m5stack-core2/src/esp_dsp_fft.rs`.

`components/mfsk_ft8/CMakeLists.txt` minimal example:

```cmake
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES espressif__esp-dsp)
add_library(mfsk_ft8_rust STATIC IMPORTED)
set_target_properties(mfsk_ft8_rust PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/libmfsk_ft8.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE mfsk_ft8_rust)
```

A worked skeleton lives at
[`embedded-poc/idf-component/`](https://github.com/jl1nie/mfsk-core/tree/main/embedded-poc/idf-component).

## What we don't ship

mfsk-core stops at the decode/encode pipeline. The following are
**deliberately out of scope** because hardware variation makes a
generic interface unhelpful:

- Audio capture (I2S, microphone gain, sample-rate clock recovery)
- Display / UI (TFT, OLED)
- Networking (Wi-Fi, BLE, MQTT)
- RTOS task wiring
- Time / clock synchronisation (NTP, GPS)
- Persistent storage / settings

`embedded-poc/m5stack-core2/src/main.rs` shows one way to wire all
of those (using esp-idf-svc) for one specific board — it is an
**example binary**, not a maintained application. Other targets
should expect to write their own glue. Reference what's there as a
template; copy what's useful.

## Performance benchmark

Three on-air recordings baked in as WAV assets (12 kHz / mono / i16
PCM, ≈ 360 KB each), decoded by the `rx-wavsim` streaming bench which
pumps them into the queue pipeline at real-time pace and decodes one
slot per WAV-completion notify. **post-SlotEnd** = wall-clock from
SlotEnd notify to "decode done" — i.e. user-perceivable RX latency
(stage 2 runs during the tail of audio capture, hidden from this
budget; see "Streaming RX pipeline architecture" below).

`q_thresh = 12` (production default, full recall).

`qso3_busy.wav` is the **WSJT-X formally-distributed FT8 reference
recording** (`samples/FT8/210703_133430.wav`, busy 7-station slot;
verified bit-identical via `cmp` 2026-05-04). `qso1` / `qso2` are
informational on-air captures — useful as breadth but not formal
reference.

| WAV | S3 LX7 post-SlotEnd (0.6.2) | decoded |
|---|---:|---:|
| qso1 (mid-band)                        | (informational) | 3 |
| qso2 (mid-band)                        | (informational) | 3 |
| **qso3 busy band (WSJT-X reference)**  | **≈ 1.19 s** | **6 / 18 JTDX + 1 bonus = 7** |

Recall on the 0.6.2 LX7 ship config (`pass1=30 max_cand=15
q_thresh=DEFAULT_Q_THRESH BP=DEFAULT_BP_MAX_ITER`, Q11i16 LLR scalar,
rectangular pre-FFT window). The 0.6.2 step lifted recall from 6 →
**7 total** (XE2X HA2NP RR73 recovered) and trimmed post-SlotEnd
from 1.341 s → 1.191 s after switching the embedded LLR scalar
back to Q11i16. The remaining 12 JTDX-confirmed signals are bounded
by the absence of `fine_refine_pass1`'s 192k-FFT cd0 chain on
Xtensa — see "Embedded fine_refine ceiling" below.

### Embedded fine_refine ceiling (0.6.2 status)

Two paths to lift embedded recall toward the JTDX-extra band were
tried during 0.6.2 development; both bottomed on the same fact:

1. **Per-symbol DFT iteration** (`fill_symbol_spectra` per cand)
   was projected at ~100 ms but actually exceeded 5 s of blocked
   compute — 200k+ × 1920-sample DFTs per slot — and tripped the
   FreeRTOS Task Watchdog.
2. **`cd0` complex-baseband via cascaded `esp-dsp` FIR decimate**
   (3:1 → 4:1 → 5:1 = 60:1) solved the compute side but ran into
   a PSRAM bandwidth ceiling: ~80 MB/s on S3 OCT mode means ~1 s
   just for the memory traffic to feed 15 candidates' worth of
   cd0 builds. `dsps_fird_init_f32`'s 16-byte alignment
   requirement, scratch fragmentation, and TLSF heap corruption
   from lazy alloc inside the decode loop were all incidental.

Embedded recall therefore stays at 7 total on `qso3_busy.wav`
until either an i16-throughout streaming FIR cd0 path or a
chunk-processing refactor lands. Both are out of scope for a
0.6.x patch; tracking under a future `decode_block_with_fine_refine`
issue.

Earlier 0.5.4-era figures here had qso3 = 7 callsigns on a 13-truth
list at 0.707 s with Hann window + the now-fixed embedded path; the
0.5.5 numbers above use the JTDX 18-entry golden as the recall axis
and the corrected rectangular window. See `CHANGELOG.md` for the
regression diagnosis.

### vs host wide-band on the WSJT-X reference

A side-by-side run of `decode_frame` (host wide-band: rustfft,
`BpAllOsd`, max_cand=200, OSD-3 fallback) vs `decode_block`
(embedded equivalent: integer pipeline, max_cand=15, q=12) on the
same `qso3_busy.wav` (test:
`mfsk-core/tests/ft8_reference_suite_recall.rs`):

| run | callsigns / 13 truth | wall-clock | hardware |
|---|---:|---:|---|
| host wide-band (`decode_frame BpAllOsd 200`) | **13 / 13** | 140 ms | Ryzen desktop |
| host fixed-point (= embedded, `decode_block` 15) | 7 / 13 | 6 ms | Ryzen desktop |
| **M5StickS3 LX7** (`decode_block`, real silicon)  | 7 / 13 | **707 ms** | post-SlotEnd, 240 MHz dual-core |
| **M5Stack Core2 LX6** (`decode_block`, real silicon) | 7 / 13 | **1434 ms** | post-SlotEnd, 240 MHz dual-core |

The 6 callsigns the embedded path misses on the busy band require the
wider PASS1=200 search + OSD-3 fallback that host wide-band runs and
the embedded budget skips. The wall-clock gap between host fixed-point
(6 ms) and embedded silicon (707 ms / 1434 ms) is the bare CPU ratio
(Ryzen ~5 GHz × 16 cores vs Xtensa 240 MHz × 2 cores) — no
algorithmic / pipeline overhead, since both run the identical integer
pipeline.

#### Why we don't widen PASS1 / enable OSD on the embedded path

Tested against the WSJT-X reference busy band on real S3 LX7 silicon
(`logs/s3_pass100_max30_2026-05-04.log`):

| config | qso3 post-SlotEnd | qso3 recall | total recall |
|---|---:|---:|---:|
| Bp/30/15 (ship)  | **0.71 s** | 7/13 | 14/22 (or 15 with phantom) |
| Bp/100/30        | **1.59 s** | 7/13 (unchanged) | +1 (qso1 OH3NIV only) |
| BpAllOsd/200/100 (host estimate) | ~7 s | 7/13 (+1 on qso3 N1JFU) | 16/22 |

Two non-obvious findings drove the decision to stay at `PASS1=30 /
max_cand=15`:

1. **qso3 busy band recall is bounded by coarse_sync rank, not BP /
   OSD effort.** Widening PASS1 from 30 → 100 + max_cand 15 → 30
   recovers no qso3 calls — the 6 missed signals are below
   coarse_sync rank 100 entirely. They need iterative subtraction
   (the WSJT-X wide-band path's hallmark) which `decode_block`
   doesn't implement.
2. **The FT8 QSO turnaround budget is ~2 s post-SlotEnd**, not the
   full 15 s slot. After decode the UI has to draw the waterfall,
   update the callsign list, render RPRT, prep next-slot TX, and —
   on chips without an NTP-synced or GPS-disciplined RTC — re-estimate
   slot timing from the **median** `dt_sec` of decoded signals (a
   plain mean is outlier-sensitive: a single bogus-sync but
   CRC-valid decode skews the slot phase noticeably; the ESP32's
   internal RTC drift is large enough that frame alignment has to
   be slaved to this decoder-derived estimate). Bp/100/30 on qso3
   leaves only ~0.4 s for all of that before the next TX must start
   — too tight. The +1 qso1-only recall gain isn't worth the
   headroom loss.

So the embedded `decode_block` ships at the recall floor that fits
the 2 s budget cleanly. Pushing further requires either (a)
porting iterative subtraction to the embedded path (open question
on cost) or (b) accepting late-arrival "spotter mode" decodes that
land too late for QSO turnaround.

Per-stage breakdown (qso3 busy band):

| stage | Core2 LX6 | S3 LX7 | notes |
|---|---:|---:|---|
| stage 1 (incremental, during capture) | ≈ 1.0 s of compute over 15 s | same | ~6 % capture CPU |
| stage 2 `coarse_sync_split_with_allsum` (during capture) | 0.65 s | 0.18 s | hidden under SlotEnd notify latency |
| pass 2 `pass2_split` (post-SlotEnd) | 0.19 s | 0.12 s | dual-core, head/tail split |
| stage 3 `stage3_split` (post-SlotEnd) | 1.24 s | 0.58 s | dual-core, **work-stealing** per-cand |

The two wall-clock improvements that put both chips in this range:

1. **Stage 2 hidden under capture.** `stage1_inc` ships its
   `SpecBundle` (spec + per-half allsums) on the spec_q queue as soon
   as pair 92 finalises (≈ 200 ms before SlotEnd), so main runs
   `coarse_sync_split_with_allsum` in parallel with the tail of audio
   capture instead of inside the post-SlotEnd budget.
2. **Stage 3 work-stealing.** `dual_core::stage3_split` does not
   pre-split candidates into head / tail. Both PRO_CPU and APP_CPU
   pull the next candidate from a shared
   `Vec<Option<RefinedCandidate>>` via `AtomicUsize::fetch_add(1)`,
   so the busier core can't stall on a slow / failing candidate
   that landed on the other side. On qso3 (where ~half of 15 cands
   fail and run all four LLR variants), this absorbs the per-cand
   BP wall-clock variance.

Raw logs:
`embedded-poc/m5stack-core2/logs/core2_q_sweep_2026-05-04.log`,
`embedded-poc/m5stack-s3/logs/s3_workstealing_2026-05-04.log`.

## Streaming RX pipeline architecture

The post-Phase-E pipeline (wired up in `embedded-poc/embedded-shared`)
is **queue-based, single-ownership per slot** — no shared mutable
state, no notify-and-out-pointer split:

```
wav_sim (PRO_CPU, prio 4)
  │
  │  ChunkMsg = Samples(Vec<i16>) | SlotEnd { wav_idx, total_samples }
  ▼
chunk_q (depth 4)
  │
  ▼
stage1_inc worker (APP_CPU, prio 3)
  │  internal: per-slot WorkerCtx { audio, spec, allsum_head/tail,
  │                                 next_pair, … }
  │  fires SpecBundle as soon as pair 92 lands (≈ 200 ms before
  │  SlotEnd) so main can start stage 2 during the tail of capture
  │
  ├──▶ spec_q (depth 2): SpecBundle { spec, allsum_head, allsum_tail }
  └──▶ slot_q (depth 2): Slot { audio, wav_idx, inc_total_us }
       (after the SlotEnd ChunkMsg)
       │
       ▼
main / decode task (PRO_CPU, prio 6)
       │  recv spec_q → stage 2 (coarse_sync_split_with_allsum, dual-core)
       │  recv slot_q → pass 2 (refine_candidates, dual-core)
       │              → stage 3 (work-stealing per-cand, dual-core)
       ▼
DecodeResult[]
```

`dual_core` exposes a separate set of FreeRTOS Queues for stage 2 /
pass 2 / stage 3 dispatch (one job queue + one per-variant result
queue). All ownership transfers via `Box::into_raw` raw-pointer items
on the queues — host-`mpsc::sync_channel`-equivalent semantics.

Pipeline invariants:
- wav_sim sends Samples / SlotEnd for one slot in FIFO order.
- stage1_inc emits SpecBundle at most once per slot (first time
  `next_pair == N_PAIRS`, or fallback in `finalize_slot` if pair 92
  never landed).
- main pairs SpecBundle ↔ Slot by FIFO order of receipt.
- main blocks on `STAGE3_RESULT_Q` recv before returning, so
  worker-side raw pointers (audio, cs scratch, work-stealing slot
  array) outlive the worker's access for the duration of the call.

See `embedded-poc/embedded-shared/src/pipeline.rs` (queue helpers +
`ChunkMsg`/`SpecBundle`/`Slot` types) and
`embedded-poc/embedded-shared/src/dual_core.rs` (the work-stealing
stage 3 dispatch + Job enum).

## Binary footprint (Core2 reference, `xtensa-esp32-elf-size -A`)

| Region | Size | Contents |
|---|---|---|
| **IRAM** (`.iram0.text` + `.iram0.vectors`) | **69 KB** | Internal-RAM code: esp-idf interrupt handlers, Wi-Fi/BT IRAM-resident routines |
| **DRAM** (`.dram0.data` + `.dram0.bss`) | **76 KB** | Internal-RAM static data: BASIS scratch (60 KB) + spectrogram cache + esp-idf statics |
| **Flash text** (`.flash.text`) | **448 KB** | App + esp-idf code |
| **Flash rodata** (`.flash.rodata`) | **1.21 MB** | Read-only data — **incl. the three baked WAVs (~1.08 MB)** for the offline real-audio bench |
| **Total app binary** | **1.997 MB** | What `espflash flash` writes |

Subtracting the baked WAV assets (1.08 MB) and the bundled esp-idf
runtime, `mfsk-core` itself plus the M5Stack Core2 example glue
contributes roughly **150–200 KB** of flash text. The IRAM/DRAM
totals shown include esp-idf — the library proper has no IRAM
requirement and the only DRAM requirement is the 60 KB BASIS scratch
plus an optional 8 KB-per-candidate cs Box (~120 KB peak across
~15 surviving candidates), comfortably fitting the on-chip 320 KB
SRAM budget on bare ESP32 (no PSRAM needed strictly for the decode
path; PSRAM helps only for the wider spectrogram).

