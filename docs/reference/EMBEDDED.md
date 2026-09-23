# Embedded targets

`mfsk-core` is `no_std + alloc` capable: the FT8 decode path
(`mfsk_core::ft8::decode_block`) runs on chips with as little as
~150 KB of usable RAM when paired with a caller-supplied FFT backend.
FST4, FT4 and WSPR reach hardware too, each by a different route. This
document is the reference for embedded integrators — what the library
asks of the caller, what scratch is needed, and what performance to
expect on the targets we exercise.

## Where to start, by intent

- **Integrating `mfsk-core` on a new MCU** → [the FFT extern
  contract](#the-fft-extern-rust-contract), then [Cargo features for
  embedded use](#cargo-features-for-embedded-use).
  [`embedded-poc/embedded-shared/src/esp_dsp_fft.rs`](https://github.com/jl1nie/mfsk-core/blob/main/embedded-poc/embedded-shared/src/esp_dsp_fft.rs)
  is the worked example to copy from.
- **Calling from C, or from a non-Rust ESP-IDF project** →
  [§ Calling from C](#calling-from-c-or-another-non-rust-project), and
  [`BINDINGS.md`](BINDINGS.md) for the ABI itself.
- **Operating an existing controller** →
  [`MANUAL_M5STACK_CORES3.md`](MANUAL_M5STACK_CORES3.md) (CoreS3, takes
  audio from a radio over USB) or
  [`MANUAL_M5STICKS3.md`](MANUAL_M5STICKS3.md) (the StickS3 demo board).
- **Contributing to one of the embedded apps** →
  [`embedded-poc/CLAUDE.md`](https://github.com/jl1nie/mfsk-core/blob/main/embedded-poc/CLAUDE.md)
  for cross-board toolchain notes and the LX6/LX7 comparison, then the
  per-crate `CLAUDE.md` for board-specific gotchas.
- **WSPR rather than FT8** → [WSPR on embedded](#wspr-on-embedded).
- **Tracking the roadmap** → [`ROADMAP.md`](../notes/ROADMAP.md)
  Phase B-Stick (StickS3 demo / acoustic fallback), Phase B-Core
  (CoreS3 main UAC controller) and Phase E (WSPR).
- **Host-only usage** → [`LIBRARY.md`](LIBRARY.md).

## Contents

- [Architecture: how f32 and fixed-point share one codebase](#architecture-how-f32-and-fixed-point-share-one-codebase)
- [What we test](#what-we-test)
- [Cargo features for embedded use](#cargo-features-for-embedded-use)
- [The FFT extern Rust contract](#the-fft-extern-rust-contract)
- [Per-symbol DFT: Goertzel](#per-symbol-dft-goertzel)
- [Q-format quick reference](#q-format-quick-reference)
- [Calling from C or another non-Rust project](#calling-from-c-or-another-non-rust-project)
- [What we don't ship](#what-we-dont-ship)
- [Performance benchmark](#performance-benchmark)
- [Streaming RX pipeline architecture](#streaming-rx-pipeline-architecture)
- [Binary footprint](#binary-footprint-core2-reference-xtensa-esp32-elf-size--a)
- [Per-protocol embedded status](#per-protocol-embedded-status)
- [WSPR on embedded](#wspr-on-embedded)

## Architecture: how f32 and fixed-point share one codebase

The whole DSP / FEC pipeline is parameterised by **scalar traits**, so
the same source compiles to either a host-friendly f32 path or an
embedded-friendly integer path **with no duplicated code**:

- [`engine::scalar::SpecScalar`] — spectrogram / DFT-output scalar
  (`f32` on host; `Q14i16` for embedded cs storage).
- [`engine::scalar::LlrScalar`] — LLR scalar with wide-accumulator
  type: `f32` on host, and **`Q11i16` with an i32 wide accumulator**
  under `fixed-point-llr`. [^llrwidth]
- [`engine::scalar::Cmplx<S>`] — generic complex over a `SpecScalar`.
  As of 0.6.3 (cleanup β.5) this is a type alias for
  `num_complex::Complex<S>`, so the embedded integer path and the
  host f32 path share the same complex algebra implementation.
- `compute_llr_generic<P, S, T>`, `compute_snr_db_generic<P, S>`,
  `bp_decode_generic_nms<P, T>` — all take the scalar types as
  generic parameters; one monomorphisation per `(P, S, T)` triple.

The `fixed-point` Cargo feature just **swaps which scalar types the
protocol glue picks**; the generic body is unchanged. This means the
embedded port shares 99 % of its code with the host build — bug fixes
and optimisations land once and apply everywhere.

[^llrwidth]: Why Q11i16 and not narrower. The LLR type was `Q3i8` in
    0.5.x, and 0.6.2 widened it: on `qso3_busy.wav`, host fixed-point
    with rustfft reached 16/18 in f32 but only **9/18** with `Q3i8`,
    whose ~0.875 quantisation step was the recall ceiling — not
    anything DSP-side. `Q11i16` (~1/2048 LSB) closes that gap fully on
    host; on real silicon it is worth one entry (6/18 → 7), the rest of
    the host gap being NSTEP-half, coarse-sync simplifications and the
    absent `fine_refine_pass1` (the CoreS3 per-slot pipeline now has an
    equivalent — see [Per-slot decode on the
    CoreS3](#per-slot-decode-on-the-cores3-fine-sync-retries-the-key-up-bound)). BP scratch doubles from ~6 KB to
    ~12 KB, still inside the S3 / Core2 internal-DRAM budget. `Q3i8`
    stays in `engine::scalar` for the comparison path. (The sweep above
    predates the 0.6.3 OSD tightening, which later dropped 3 CRC-luck
    phantoms from f32 host recall, 16/18 → 13/18.)

### What the fixed-point switch is wired up to today

| Component | Generic over | Fixed-point switch wired? |
|---|---|---|
| LDPC BP NMS (`fec::ldpc::bp`) | `LlrScalar` | ✅ via **`fixed-point-llr`** — a separate, opt-in feature since #349; on an LX7 the i16 loop measures 0.85× f32 |
| LLR computation (`engine::llr`) | `SpecScalar` × `LlrScalar` | ✅ via `fixed-point` (spectra) × `fixed-point-llr` (LLR type) |
| BP scratch pool (`BpScratch<P, T>`) | `LdpcParams` × `LlrScalar` | ✅ — works for FT8 LDPC(174,91) and FST4/uvpacket LDPC(240,101) |
| FT8 spectrogram + DFT (`ft8::decode_block`) | `SpecScalar` × `AudioSample` | ✅ via `fixed-point` |
| WSPR (`wspr::decode`, `wspr::ddc`) | — | ❌ — runs plain host f32 on embedded too, via `fft-extern`; never needed the integer path. See [WSPR on embedded](#wspr-on-embedded) below. |
| **FT4** | (host f32 only) | ❌ — and it does not need to be. FT4 routes through the generic `engine::pipeline`, like FST4; `fixed-point` would be a no-op on that path, and on LX7 it measured *slower* than f32 anyway (issue #198). It now **builds and decodes on hardware** — see [Per-protocol embedded status](#per-protocol-embedded-status). |
| **Q65 / JT9 / JT65** | (host f32 only) | ❌ — they build under `alloc,<mode>,fft-extern` since #390, but nothing embedded drives them yet: no fixed-point path, no app |

So: **the trait infrastructure is protocol-agnostic, but the only
protocol that actually flips into the integer path on the embedded
build is FT8.**

**Flipping into the integer path is not what "runs on a chip" means.**
The generic `engine::pipeline` is itself an embedded route: FST4
reached hardware through it without any `decode_block` port (issue
#306), and FT4 has now done the same. `decode_block` exists because
FT8's own downsample chain needs a 192 000-point FFT — it is a way
around one specific FFT, not the definition of embeddability. See
[Per-protocol embedded status](#per-protocol-embedded-status).

WSPR reached embedded by a different route entirely (below), worth
noting here since the table above could otherwise be read as "only
FT8 runs on a chip": it doesn't need `decode_block`'s spectrogram/DFT
machinery or the integer pipeline, because the S3's dual-core headroom
covers WSPR's much slower cadence (a 120 s slot vs FT8's ~1.2 s
post-SlotEnd target) comfortably in plain f32.

## What we test

| Target | MCU | Backend | Status |
|---|---|---|---|
| **M5StickS3** | **ESP32-S3 (Xtensa LX7 dual-core, 240 MHz, 8 MB Octal PSRAM, ES8311 codec, ST7789P3 135×240 LCD, KEY1/KEY2)** | esp-dsp `_ae32_` asm (LX6/LX7 shared, scalar single-issue) — LX7 PIE `_aes3_` migration pending, see [`PHASE_D_PIE_SIMD.md`](../notes/PHASE_D_PIE_SIMD.md) | **Demo / acoustic-fallback controller** (2026-05-17 pivot — the board cannot source VBUS for USB host; the S3 silicon can host, the board does not wire the power path) — `embedded-poc/m5stack-s3-app/` (LCD UI + QSO FSM + BLE CI-V + acoustic mic + WiFi UDP log). |
| **M5Stack Core2** | **ESP32-D0WD-V3** (Xtensa LX6, dual-core 240 MHz, single-issue f32 FPU, 16 MB flash, ~4 MB PSRAM) — confirmed by `espflash board-info`: `Chip type: esp32 (revision v3.1)` / `Features: WiFi, BT, Dual Core, 240MHz`. **Not** an ESP32-S2 (LX7, single-core, no BT) or S3. | esp-dsp ASM (`dsps_dotprod_s16_ae32`, `dsps_fft2r_*`) | **Production app (`wav_sim` only)** — `embedded-poc/m5stack-core2-app/` runs the same `decode_block` against the baked `wav_sim` audio loop on LX6 to cross-validate the `mfsk-app-shared` API. Classic ESP32 has no USB peripheral, so live mic / speaker / USB-Host paths are not on the table for this board — Core2's role is the second-board LX6 verifier for the shared QSO FSM. (The original standalone Core2 compute bench `embedded-poc/m5stack-core2/` was retired in #61 Phase 3 once this app crate covered the same wav_sim path in production-app shape.) |
| ESP32-S3 compute bench | Xtensa LX7 | esp-dsp ASM | **Timing-regression bench** — `embedded-poc/m5stack-s3/`, drives `decode_block` against canned WAV inputs for per-stage timing sweeps. Not for end users. |
| **M5Stack CoreS3** | ESP32-S3 LX7 + AXP2101 PMIC + AW9523B I/O expander (USB host VBUS needs port1 bit7 `BOOST_EN` + port0 bit5 `USB_OTG_EN` + port0 bit1 `BUS_OUT_EN`, all three — see `embedded-poc/CLAUDE.md` "USB host VBUS on CoreS3") | esp-dsp `_ae32_` asm (same Phase D D1 migration applies) | **Main UAC controller target** (Phase B-Core, 2026-05-17 pivot) — `embedded-poc/m5stack-cores3-app/`. Phase 0-Core (bringup) + Phase 1-Core (AW9523B BUS_OUT_EN + UAC host) shipped in commit `1a93c92`. M5StickS3 cannot source VBUS for USB-OTG host (silicon can host, board has no VBUS source circuit — see issue #360), so it was repositioned as the **demo / acoustic-fallback** board and the live USB Audio Class path to IC-705 lands on CoreS3 instead. See `docs/notes/ROADMAP.md` Phase B-Core. |

### Other targets — what's verified vs aspirational

The `fft-extern` contract is *designed* to be target-portable, and the
`no_std` feature set cross-builds to several non-Xtensa MCUs:

| Target | builds | FFT shim shipped | Hardware-tested |
|---|---|---|---|
| `xtensa-esp32-espidf` | ✅ | ✅ esp-dsp (Core2) | ✅ qso1/2/3 sweep |
| `xtensa-esp32s3-espidf` | ✅ | ✅ esp-dsp (S3 bench + S3-app + CoreS3-app) | ✅ live off-air |
| `thumbv8m.main-none-eabihf` (RP2350 Cortex-M33) | ✅ [^xbuild] | ❌ candidates: CMSIS-DSP via pico-sdk-rs | ❌ |
| `riscv32imac-unknown-none-elf` (RP2350 Hazard3) | ✅ [^xbuild] | ❌ no DSP library; `microfft` for FFT | ❌ |
| `thumbv7em-none-eabihf` (Cortex-M4F / M7) | not tried | ❌ candidates: CMSIS-DSP `arm_*_q15` | ❌ |
| `thumbv6m-none-eabi` (Cortex-M0+ / RP2040) | not tried | ❌ scalar Rust only (no DSP unit) | ❌ |

[^xbuild]: The two RP2350 rows were verified against the retired
    `mfsk-ffi-ft8` crate and have **not** been re-checked since it was
    removed in 0.11.0. `mfsk-core` itself is what builds now, and the
    feature set is exercised on every push by
    `scripts/pre-push-check.sh`'s matrix (`alloc ft8 fft-extern` and
    `alloc ft8 fft-extern fixed-point`) — on the host target only.
    Treat the two ✅s as "expected to work", not as a current
    measurement.

**ESP32 / ESP32-S3** (Xtensa LX6 / LX7) are the only targets we
exercise end-to-end with real audio. For anything else:

```sh
cargo build -p mfsk-core --release --no-default-features \
    --features alloc,ft8,fft-extern,fixed-point --target <T>
```

and supply the FFT extern Rust symbol yourself. Concrete RP2040 /
RP2350 / Cortex-M shims are tracked as future work.

`embedded-poc/embedded-shared/src/esp_dsp_fft.rs` is the worked
example to copy from.

## Cargo features for embedded use

Default features include `std`, `parallel`, and `fft-rustfft` — turn
those off and pick the embedded baseline:

```toml
[dependencies]
mfsk-core = { version = "0.11", default-features = false, features = [
    "alloc",            # Vec / Box / String — required for decode
    "ft8",              # FT8 protocol glue
    "fft-extern",       # caller supplies the FFT backend
    "fixed-point",      # u16 spec + i16 DFT (LLR/BP are f32 since #349;
                        # add "fixed-point-llr" for the i16 hot loop)
    # Optional:
    # "profile-coarse", # always-on stage-2 sub-stage timing
] }
```

**`fixed-point` implies `nstep-half`** (since 0.6.4): the embedded
build uses NSTEP = NSPS/2 = 960 samples per spectrogram column,
whereas the host default is the WSJT-X-faithful NSPS/4 = 480. The
two features were always co-enabled on every embedded target in
practice; coupling them ensures host fixed-point builds faithfully
simulate the embedded time-grid.

Stage-3 sensitivity is a runtime parameter on
`process_candidates_into` (`q_thresh: u32`), not a Cargo feature.
[`mfsk_core::ft8::decode_block::DEFAULT_Q_THRESH`] is 12 — full
recall on every target we currently ship for. Lowering it (e.g.
q=6) widens the search at modest wall-clock cost; raising it (q=14)
saves only 0–78 ms on `qso3_busy` while losing one weak decode per
target. Keep `q_thresh` at the default for production.

Feature reference:

| Feature | What it changes | When to use |
|---|---|---|
| `std` | Pulls in `std::env`, `std::time::Instant`. Decoupled from rustfft. | esp-idf-svc-style targets that have std. Optional on bare-metal. |
| `alloc` | `extern crate alloc` + Vec / Box. | All decode paths. |
| `fft-extern` | FFT backend via `mfsk_core_make_default_fft_planner` extern fn (and the i16 variant `_planner_16`). | Any embedded target. |
| `fft-rustfft` | rustfft as the FFT backend. | Host only. |
| `fixed-point` | Embedded integer pipeline: u16 spectrogram + i16 internal DFT. Implies `nstep-half`. **No longer implies the Q11i16 LLR/BP hot loop** — that is `fixed-point-llr`, opt-in since #349, because on an LX7 the i16 BP measures 0.85× f32 (22 813 vs 19 456 µs) and what `fixed-point` actually defends is the spectrogram's 702 → 351 KB. (Was `Q3i8` in 0.5.x — 0.6.2 widened the LLR to `Q11i16` because host fixed-point + rustfft hit 16/18 on `qso3_busy.wav` with f32 but only 9/18 with `Q3i8`; the resolution step was the recall ceiling, not anything DSP-side. `Q3i8` stays in `engine::scalar` for the comparison path.) | Any embedded target — close to host f32 recall (1/2048 LSB LLR resolution), halved PSRAM bandwidth, ~12 KB BP scratch (Q11i16, post-0.6.2). |
| `nstep-half` | NSTEP = NSPS/2 (vs WSJT-X-faithful NSPS/4) for the spectrogram column rate. | Auto-enabled by `fixed-point`. Don't enable independently on a host build unless you're explicitly simulating the embedded path. |
| `parallel` | Rayon-parallel candidate processing. | Host only. Always off on embedded (no `std::thread`). |
| `profile-coarse` | Always emits coarse_sync sub-stage timings to stderr. | Diagnosis only. |

## The FFT extern Rust contract

`mfsk_core::engine::fft::FftPlanner` (and `FftPlanner16` for the i16
path) is the decode path's FFT trait. Under `fft-extern`, the library
expects the linked binary to provide two `extern "Rust"` factory
functions:

<!-- Not compiled: `MyEspDspPlanner`/`MyEspDspPlanner16` are stand-ins
     for whatever backend type the downstream binary defines, and this
     block deliberately shows the binary side of a weak-linkage extern
     contract rather than something the library crate can exercise
     itself — matching `engine::fft::default_planner`'s own doc
     comment, which marks its (shorter) version of this same example
     `ignore` for the same reason. -->

```rust,ignore
#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_make_default_fft_planner()
    -> Box<dyn mfsk_core::engine::fft::FftPlanner>
{
    Box::new(MyEspDspPlanner::new())
}

#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_make_default_fft_planner_16()
    -> Box<dyn mfsk_core::engine::fft::FftPlanner16>
{
    Box::new(MyEspDspPlanner16::new())
}
```

`embedded-poc/embedded-shared/src/esp_dsp_fft.rs` is a working
example that bridges to esp-dsp's Xtensa ASM kernels
(`dsps_fft2r_fc32_ae32` + `dsps_fft2r_sc16_ae32` for the i16 path).
RP2040 / Cortex-M implementations would bridge to CMSIS-DSP
similarly.

## Per-symbol DFT: Goertzel

The FT8 per-symbol DFT evaluates `Σ x[n] · exp(-jωn)` at each of the
8 tone frequencies across NSPS = 1920 samples, for every (symbol,
tone) pair in every candidate's `cs` matrix (79 symbols × 8 tones =
632 DFTs per candidate, ~15 candidates per slot ⇒ ~9.5k DFTs per
slot). The implementation lives in
`mfsk-core/src/ft8/decode_block/fill_symbol_spectra.rs`:

### Goertzel — `fill_symbol_spectra_goertzel`

A generalised Goertzel recursion: per (sym, tone) a 2-tap IIR with
3 f32 state values that get thrown away on return. **Zero caller
scratch**, zero internal-DRAM static buffers, zero extern symbols
required. This has been the sole path for embedded callers since
0.6.4; the legacy BASIS (Q15 sin/cos dot-product) fill path it
replaced was removed entirely in 0.8.0 (issue #162).

The performance trick: order the loop **sample-outer / tone-inner**
so the 8 per-tone recursions (one per FT8 tone) run as 8 independent
dependent chains through the FPU pipeline. LLVM unrolls the
constant-bound `NTONES = 8` inner loop and the Xtensa FPU absorbs
~all of the per-chain latency in parallel. Result: stage-3 cost
matches the old BASIS asm dot product (~1.4 s on S3 `qso3_busy.wav`)
**with zero scratch + +0.16..+0.63 dB SNR improvement** over BASIS
(f32 Goertzel has more precision than Q15 BASIS had).

Why BASIS was retired: it needed a precomputed Q15 sin/cos table
(`BASIS_RE` / `BASIS_IM`, each `NTONES × NSPS = 15 360` i16 entries
≈ 30 KB) living in fast internal SRAM (not PSRAM) for the ASM dot
product to hit its rated throughput — 30 KB per axis × 2 axes ×
2 cores = **120 KB of internal DRAM**, exactly what the M5StickS3
Qso-mode bidirectional I2S DMA descriptor needed to allocate. The
board's free contiguous internal chunk couldn't satisfy both, and
Qso mode boot failed with `i2s_alloc_dma_desc: allocate DMA buffer
failed`. Goertzel freed that 120 KB without losing perf, and 0.8.0
finished the job by deleting the now-dead BASIS code and its
`basis_re`/`basis_im` scratch parameters outright — new integrations
never need to think about scratch placement here at all.

## Q-format quick reference

| Stage | Format | Range | File |
|---|---|---|---|
| Spectrogram cell | u16 (mag²) | `>> FP_SPEC_SHIFT (12)`, saturated since 0.6.4 | `ft8::decode_block::spectrogram::Spectrogram` |
| Symbol cs | `Cmplx<f32>` (default) or `Cmplx<Q14i16>` (`fixed-point`) | f32 unbounded; Q14 ±2 | `engine::scalar::Cmplx` (type alias for `num_complex::Complex`) |
| LLR | f32 (host **and embedded by default since #349**) or **Q11i16** (`fixed-point-llr`, opt-in; the type has been Q11i16 since 0.6.2 — was `Q3i8` in 0.5.x, widened to address the resolution-limited recall ceiling) | f32 unbounded; Q11i16 ±16 with ~1/2048 LSB (Q3i8 ±16 with ~1/8 LSB stays in `engine::scalar` for the comparison path) | `engine::scalar::LlrScalar` |
| BP messages | T (same as LLR) | — | `fec::ldpc::bp::bp_decode_generic_nms_with_scratch` |

## Calling from C or another non-Rust project

**A Rust shim is required either way.** `mfsk-core`'s embedded FFT
contract is an `extern "Rust"` symbol
(`mfsk_core_make_default_fft_planner`), which is a different ABI from
`extern "C"` and cannot be satisfied by a C translation unit. Any
non-Rust integration therefore links at least a small Rust staticlib.

Once a consumer is writing Rust anyway, calling `mfsk-core` directly is
strictly simpler than routing through a C ABI — which is why
**`mfsk-ffi-ft8`, the FT8-only embedded C ABI, was retired in 0.11.0**.
All three boards in `embedded-poc/` call `mfsk-core` directly. That is
the recommended shape:

```text
your-app/                      # esp-idf project root
├── main/main.c                # your application
├── components/mfsk/
│   ├── CMakeLists.txt         # IMPORTED static-lib component
│   └── lib/libyourshim.a      # from the Rust build below
└── shim/                      # Rust staticlib
    ├── Cargo.toml             # depends on mfsk-core
    ├── .cargo/config.toml     # target = xtensa-esp32s3-espidf, panic=abort
    └── src/lib.rs             # #[no_mangle] extern "C" entry points you
                               # define, plus the extern "Rust" FFT planner
```

Build the shim with the Xtensa toolchain:

```sh
source ~/export-esp.sh
RUSTFLAGS="-C panic=abort" cargo build --release \
    --target xtensa-esp32s3-espidf          # or xtensa-esp32-espidf
```

`-C panic=abort` is required: Rust unwinding panics need `std`.
ESP-IDF projects usually set it in `.cargo/config.toml`:

```toml
[target.xtensa-esp32s3-espidf]
rustflags = ["-C", "link-arg=-nostartfiles", "-C", "panic=abort"]
```

and import the archive as a component:

```cmake
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES espressif__esp-dsp)
add_library(mfsk_rust STATIC IMPORTED)
set_target_properties(mfsk_rust PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/libyourshim.a)
target_link_libraries(${COMPONENT_LIB} INTERFACE mfsk_rust)
```

**If you want the full C ABI** rather than hand-written entry points,
`mfsk-ffi` builds as a staticlib for these targets too and covers every
protocol — see [`BINDINGS.md`](BINDINGS.md). It is larger than an
FT8-only shim and pulls the session/stream machinery, so for a
single-protocol MCU build a hand-written shim is usually smaller.

[`embedded-poc/idf-component/README.md`](https://github.com/jl1nie/mfsk-core/blob/main/embedded-poc/idf-component/README.md)
writes this out at length — the shim's `Cargo.toml`, the CMake
component, the build flow, and both linking options. It is
documentation rather than a buildable skeleton: the files it used to
ship were built around `mfsk-ffi-ft8` and were not carried forward.

### Streaming capture: I2S / USB Audio → 12 kHz

A decode takes one whole slot at 12 kHz. Real receivers give you small
DMA chunks at whatever rate the codec runs — typically 16 / 24 / 48 kHz
from I2S or USB Audio Class 1/2. Two options:

- **From Rust**, `engine::dsp::resample` converts to 12 kHz and your
  app owns the ring. `embedded-poc/embedded-shared/src/pipeline.rs` is
  the worked example, and [its architecture is
  below](#streaming-rx-pipeline-architecture).
- **From C**, the `mfsk_stream_*` family in `mfsk-ffi` is exactly this
  ring, sized from the mode's own slot length and driven without the
  library reading any clock — see
  [`BINDINGS.md` §2.5](BINDINGS.md#25-streaming-capture).
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

The `embedded-poc/` crates show one way to wire all of those (using
esp-idf-svc) for two specific boards:

- `embedded-poc/m5stack-s3-app/` — M5StickS3 FT8 controller
  (ES8311 acoustic mic, BLE CI-V to IC-705, LCD UI, QSO FSM, optional
  WiFi UDP log). Production, daily-use target.
- `embedded-poc/m5stack-core2-app/` — Core2 (LX6) sibling, runs the
  decoder against a baked `wav_sim` audio loop with the LCD wired
  up. External I/O deferred. Used to cross-validate the
  `mfsk-app-shared` API on LX6.

Both are **examples**, not maintained applications you're expected
to fork without changes. Reference what's there as a template; copy
what's useful.

## Performance benchmark

Three on-air recordings baked in as WAV assets (12 kHz / mono / i16
PCM, ≈ 360 KB each), decoded by the `rx-wavsim` streaming bench
which pumps them into the queue pipeline at real-time pace and
decodes one slot per WAV-completion notify. **post-SlotEnd** =
wall-clock from SlotEnd notify to "decode done" — i.e.
user-perceivable RX latency (stage 2 runs during the tail of audio
capture, hidden from this budget; see "Streaming RX pipeline
architecture" below).

`q_thresh = 12` (production default, full recall).

`qso3_busy.wav` is the **WSJT-X formally-distributed FT8 reference
recording** (`samples/FT8/210703_133430.wav`, busy 7-station slot;
verified bit-identical via `cmp` 2026-05-04). `qso1` / `qso2` are
informational on-air captures — useful as breadth but not formal
reference.

S3 LX7 numbers below are from the 0.6.3 Q11i16 ship sweep
(`embedded-poc/m5stack-s3/logs/` archived development run on
2026-05-09; the raw log file was not preserved in the repo —
only the 0.6.2 → 0.6.3 Q3i8 → Q11i16 phase logs remain under
`logs/s3_phaseA..C_q3i8_2026-05-04.log` etc.). 0.6.4 Goertzel
preserves these wall-clocks and adds +0.16..+0.63 dB SNR on the
same decodes; re-measuring on 0.6.5 firmware is the right way to
confirm post-0.6.3 OSD-tightening did not move the embedded numbers.

| WAV | S3 LX7 post-SlotEnd | decoded |
|---|---:|---:|
| qso1 (mid-band)                        | **1.10 s** | 3 |
| qso2 (mid-band)                        | **1.68 s** | 4 |
| **qso3 busy band (WSJT-X reference)**  | **1.19 s** | **7 / 18 JTDX** |

### vs host wide-band on the WSJT-X reference

A side-by-side run of `decode_frame` (host wide-band: rustfft,
`DecodeDepth::FULL`, max_cand=200, OSD-3 fallback) vs `decode_block`
(embedded equivalent: integer pipeline, max_cand=15, q=12) on the
same `qso3_busy.wav`:

| run | callsigns / 18 JTDX truth | wall-clock | hardware |
|---|---:|---:|---|
| host wide-band (`decode_frame DecodeDepth::FULL 200`) | **16 / 18** | ~140 ms | Ryzen desktop |
| host fixed-point (= embedded, `decode_block` 15) | 7 / 18 | ~6 ms | Ryzen desktop |
| **M5StickS3 LX7** (`decode_block`, real silicon)  | 7 / 18 | **1.19 s** | post-SlotEnd, 240 MHz dual-core |
| **M5Stack Core2 LX6** (`decode_block`, real silicon) | 7 / 18 | ~2.8 s | post-SlotEnd, 240 MHz dual-core |

The 11 callsigns the embedded path misses on the busy band require
the wider PASS1=200 search + iterative subtraction + OSD-3 fallback
that host wide-band runs and the embedded budget skips. The
wall-clock gap between host fixed-point (6 ms) and embedded silicon
(1.19 s / 2.8 s) is the bare CPU ratio (Ryzen ~5 GHz × 16 cores vs
Xtensa 240 MHz × 2 cores) — no algorithmic / pipeline overhead,
since both run the identical integer pipeline.

The 11 callsigns the embedded path misses are not a tuning oversight:
widening PASS1 and enabling OSD was measured and rejected, because the
missed signals sit below coarse_sync rank 100 entirely and need
iterative subtraction rather than more BP effort — and because the FT8
turnaround budget is ~2 s post-SlotEnd, not the full slot. The numbers
are in [`DESIGN_RATIONALE.md`
§5](../notes/DESIGN_RATIONALE.md#5-why-the-embedded-path-doesnt-widen-pass1-or-enable-osd).

### Per-stage breakdown on `qso3_busy.wav`

| stage | Core2 LX6 | S3 LX7 | notes |
|---|---:|---:|---|
| stage 1 (incremental, during capture) | ≈ 1.0 s of compute over 15 s | same | ~6 % capture CPU |
| stage 2 `coarse_sync_split_with_allsum` (during capture) | 0.65 s | 0.16 s | hidden under SlotEnd notify latency |
| pass 2 `pass2_split` (post-SlotEnd) | 0.19 s | 0.12 s | dual-core, head/tail split |
| stage 3 `stage3_split` (post-SlotEnd) | ≈ 2.5 s | 1.06 s | dual-core, **work-stealing** per-cand |

The two wall-clock improvements that put both chips in this range:

1. **Stage 2 hidden under capture.** `stage1_inc` ships its
   `SpecBundle` (spec + per-half allsums) on the `spec_q` queue as
   soon as pair 92 finalises (≈ 200 ms before SlotEnd), so main
   runs `coarse_sync_split_with_allsum` in parallel with the tail
   of audio capture instead of inside the post-SlotEnd budget.
2. **Stage 3 work-stealing.** `dual_core::stage3_split` does not
   pre-split candidates into head / tail. Both PRO_CPU and APP_CPU
   pull the next candidate from a shared
   `Vec<Option<RefinedCandidate>>` via `AtomicUsize::fetch_add(1)`,
   so the busier core can't stall on a slow / failing candidate
   that landed on the other side. On qso3 (where ~half of 15 cands
   fail and run all four LLR variants), this absorbs the per-cand
   BP wall-clock variance.

## Streaming RX pipeline architecture

The post-Phase-E pipeline (wired up in
`embedded-poc/embedded-shared/src/`) is **queue-based,
single-ownership per slot** — no shared mutable state, no
notify-and-out-pointer split:

```text
wav_sim / I2S capture (PRO_CPU, prio 4)
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
  └──▶ slot_q (depth 2): Slot { audio, wav_idx, inc_total_us, slotend_us }
       (after the SlotEnd ChunkMsg)
       │
       ▼
main / decode task (PRO_CPU, prio 6)
       │  recv spec_q → stage 2 (coarse_sync_split_with_allsum, dual-core)
       │              → fine sync (fine_sync_12k, dual-core; CoreS3 only)
       │              → pass 2 + stage 3 on the audio prefix, "ready"
       │                candidates; then coarse retries in the tail
       │                window, stopping when slot_q holds the slot
       │  recv slot_q → pass 2 + stage 3 on the full slot, "deferred"
       │                candidates → remaining coarse retries
       │              → one row per message
       ▼
DecodeResult[]
```

`dual_core` exposes a separate set of FreeRTOS Queues for stage 2 /
pass 2 / stage 3 dispatch (one job queue + one per-variant result
queue). All ownership transfers via `Box::into_raw` raw-pointer
items on the queues — host-`mpsc::sync_channel`-equivalent
semantics.

Pipeline invariants:
- The capture task sends Samples / SlotEnd for one slot in FIFO
  order.
- `stage1_inc` emits SpecBundle at most once per slot (first time
  `next_pair == N_PAIRS`, or fallback in `finalize_slot` if pair 92
  never landed).
- main pairs SpecBundle ↔ Slot by FIFO order of receipt.
- main blocks on `STAGE3_RESULT_Q` recv before returning, so
  worker-side raw pointers (audio, cs scratch, work-stealing slot
  array) outlive the worker's access for the duration of the call.

See `embedded-poc/embedded-shared/src/pipeline.rs` (queue helpers +
`ChunkMsg` / `SpecBundle` / `Slot` types) and
`embedded-poc/embedded-shared/src/dual_core.rs` (the work-stealing
stage 3 dispatch + Job enum).

### Per-slot decode on the CoreS3: fine sync, retries, the key-up bound

`decode_block`'s embedded body has skipped the host's per-candidate
fine refine since 0.6.3 — upstream computes it on a baseband cut by a
192 000-point FFT the embedded planner does not carry — so candidates
were decoded from coarse sync's 3.125 Hz / 40 ms grid. The CoreS3
per-slot pipeline now runs
`mfsk_core::ft8::decode_block::fine_sync_12k` between coarse sync
and the prefix partition (`DecodeConfig::fine_sync`): `ft8b.f90`'s
Stages A/B/C on the 12 kHz audio, each Costas symbol mixed once into
60-sample bins so every stage is a re-sum.

Three things come with it, all in `dual_core::run_speculative_slot`:

- **A coarse-position retry.** A candidate that fails stage 3 at its
  refined position is tried once at its coarse one. Fine sync alone
  erased stations the coarse position decodes; the retry is what makes
  it a strict addition. Retries are the lowest-value work — 0.26-0.46
  decodes a slot from 7-13 retries on the host mirror — so the early
  path's run only in the tail window and stop claiming the moment the
  slot arrives, and the rest wait until every first attempt, deferred
  candidates included, has had its turn.
- **One row per message.** Fine sync pulls adjacent coarse cells onto
  one carrier; results are deduplicated before they reach the slot
  count, the grid-lock policy and the QSO state machine.
- **A stage-3 bound anchored to key-up** (`DecodeConfig::key_up_guard_ms`,
  `Stage3Stop`). `budget_ms` counts from the SpecBundle's arrival and
  assumed that is ≥ 1 336 ms before slot end; it measured 1 027-1 936 ms
  depending on grid position, and below 1 336 ms its deadline fell after
  this station's key-up (slot end + 0.5 s). Claiming now also stops at
  key-up minus a guard, with slot end peeked from `slot_q` before the
  slot is received. The guard was 320 ms, because a deadline only stops
  *claiming* and a candidate already in BP ran on by up to 313 ms.
  **It is 0 now**: the clearance it bought was never the constraint —
  see [The transmit period, as WSJT-X defines
  it](#the-transmit-period-as-wsjt-x-defines-it) for what the 0.5 s is
  actually spent on, and for the second deadline this bound does not
  express.

| `qso3_busy` through the CoreS3 pipeline | decodes a slot | finished after slot end |
|---|---:|---:|
| before (2026-09-16) | 6 | ~80-100 ms |
| fine sync + retries + key-up bound (2026-09-18) | **10** | 158-291 ms |

Board figures are `MFSK_CORES3_SIM` captures
(`embedded-poc/m5stack-cores3-app/logs/sim_*_2026-09-1{6,8}.log`); the
host reproduction that measured the design, call for call against the
board's own `p1/ready/defer/dec`, is
`mfsk-core/tests/ft8_embedded_pipeline_mirror.rs`.

**Fine sync is off again as of 2026-09-20**, on all three boards. The
10-a-slot figure above was measured with it on and without a deadline;
on a radio its ~292 ms comes off the front of the early path and buys
nothing back (`cut` went 0 → 3, 15, 10, 11 with three slots past
key-up). `MFSK_FT8_FINE_SYNC` still selects it and both halves are
measured, because the answer turns on a budget that is not fixed
forever. `MFSK_FT8_KEY_UP_GUARD_MS` is likewise 0 on every board now.

### The transmit period, as WSJT-X defines it

Every number the key-up bound above rests on is fixed in WSJT-X's own
source. For FT8, with the T/R period boundary at t = 0:

```text
 0.000 s  the boundary. The transmit window opens here —
          m_bTxTime = (t2p >= tx1) && (t2p < tx2) with tx1 = 0
          (widgets/mainwindow.cpp:4552, 4596). In the same pass the
          message is taken from the auto-sequencer (txMsg =
          ui->txN->text(), mainwindow.cpp:4657-4663) and PTT is
          asserted (transceiver_ptt(true), mainwindow.cpp:4711),
          gated only on fTR < 0.75 so a late start is still allowed.
 +txDelay the rig confirms PTT; ptt1Timer then waits
          Configuration::txDelay(), default 0.200 s
          (Configuration.cpp:1602), or a hard 20 ms for FT4
          (mainwindow.cpp:8252-8253), and fires startTx2()
          (mainwindow.cpp:861-862).
 0.500 s  audio begins. Modulator::start pads silence so the waveform
          lands exactly here: delay_ms = 500 for FT8, 300 for FT4,
          1000 otherwise (Modulator/Modulator.cpp:71-74). A late start
          is TRUNCATED rather than shifted —
          m_ic = (mstr - delay_ms) * frameRate / 1000
          (Modulator.cpp:94-97) — so the grid is never given up.
          The decoder agrees: xdt = xdt - 0.5 (lib/ft8_decode.f90:210).
13.140 s  audio ends: 79 x 1920 / 12000 = 12.64 s.
13.640 s  m_bTxTime closes. tx_duration("FT8") = 1.0 + 12.64
          (helper_functions.cpp:7) — a 1 s guard past audio end, not
          extra transmission.
15.000 s  the period ends.
```

Two things follow, and they are easy to get backwards.

**The 0.5 s belongs to the transceiver, not to the decoder.** WSJT-X
spends it on PTT assert, rig turnaround and modulator padding, having
already committed the message at the boundary. `txDelay` is *inside*
that 0.5 s, not a lead added to it.

**So a decode finishing inside the 0.5 s is too late to be answered
this period**, even though `FT8_KEY_UP_AFTER_SLOT_END_US` lets stage 3
run there. There are two deadlines, and this pipeline only encodes
one: stage 3 stops claiming at slot end + 0.5 s, while *deciding what
to send* has to be done by slot end. On the air (2026-09-19, 118 slots,
`logs/udp_ts_2026-09-19.log`) `post_slotend` ran median 332 ms /
p90 479 ms — inside the encoded deadline throughout, and past the reply
deadline on most slots. Nothing is wrong today because this board does
not transmit yet; when it does, the number to watch is `post_slotend`
against **0**, not against 500.

**That conclusion assumes the message is committed at the boundary**,
which is what WSJT-X's GUI does rather than what the air requires. The
waveform is the first thing that needs the message; PTT does not, and
this board keys its radio by VOX on the USB audio, so there is no PTT
step at all. The FT4 section below works that through. It has **not**
been re-evaluated for FT8, where it would move the reply deadline from
slot end to the audio start at slot end + 0.5 s — i.e. onto the bound
`FT8_KEY_UP_AFTER_SLOT_END_US` already encodes.

Keeping the 0.5 s is still right: stage 3's late path needs the full
slot, and the full slot does not exist until the boundary. The 0.5 s is
what makes a late path possible at all — it is borrowed from the
transmitter, and a transmitting build has to pay it back.

WSJT-X itself never has to choose, because `jt9` is a separate process
and `MainWindow::decode()` only declines to *start* a decode while the
previous one is running. This board decodes on the cores that will
drive the transmitter, which upstream does not have to consider.

### FT4: the reply is due when the audio starts

Settled 2026-09-21. FT4's 7.5 s period leaves no room to be vague
about which of the moments above binds, and an earlier reading here
called upstream "internally inconsistent" where it is not.

**What upstream fixes**, all read from source:

| where | value | meaning |
|---|---|---|
| `Modulator/Modulator.cpp:74` | `delay_ms = 300` | the waveform leaves the PC 0.300 s into the period |
| `lib/ft4/ft4sim.f90:85` | `k = nint((xdt + 0.5)/dt) - NSPS` | **DT = 0 means the first active symbol at 0.500 s**; the 105-symbol waveform carries one 48 ms ramp symbol ahead of it |
| `lib/ft4_decode.f90:462` | `xdt = ibest/666.67 - 0.5` | the decoder uses the same 0.5 s reference |
| `widgets/mainwindow.cpp:8252` | `if(m_mode=="FT4") ms_delay=20` | FT4 does not wait on a relay sequencer |
| `widgets/mainwindow.cpp:1819` | `samples = 21*3456` | a WSJT-X receiver decodes 6.048 s of the period |

So the first active symbol leaves the PC at 0.348 s, and over a path
with no latency it would decode at **DT −0.15 s**. WSJT-X sends FT4
~150 ms early. The natural reading is an allowance for the transmit
chain — sound card, rig, PTT or VOX — that FT4's forced 20 ms
`txDelay` does not otherwise absorb; upstream does not say so in a
comment, and this document should not pretend it does. Either way the
0.3 and the 0.5 are not a contradiction: one is when the audio leaves,
the other is where the frame is meant to land.

**The message has to exist when the waveform starts**, and not before.
WSJT-X commits it at the boundary because `guiUpdate` reads the message
in the same pass that raises PTT (`mainwindow.cpp:4657-4711`), but PTT
does not need the content and the waveform does. This board keys the
IC-705 by **VOX on its USB audio**, so there is no separate PTT at
all: the transmitter becomes active the moment the audio does.

Measured from `ft4_rx::CAPTURE_CLOSE_SAMPLES` (6.775 s into the period
the other station transmits in):

```text
 period N — the other station transmits, this board receives
 0.000 s  their boundary; their message and PTT
 0.300 s  their audio starts (first active symbol 0.348 s)
 5.340 s  their audio ends (105 x 576 / 12 000 = 5.04 s)
 6.775 s  capture closes (0 ms)         -> candidate loop starts

 period N+1 — this board transmits
 7.500 s  boundary (725 ms)             -> nothing to do: VOX, no PTT
 7.8 - e  reply fixed (~1 025 ms)       -> encode + first USB buffer
 7.800 s  USB audio starts (1 025 ms)   -> IC-705 keys on VOX
 7.848 s  first active symbol leaves the board
~7.95 s   on air after VOX + rig latency -> DT ~ 0 at the other end
12.840 s  audio ends
```

Three consequences:

- **The reply deadline is ~1 025 ms after capture close.** Not the
  boundary's 725, and not the 1 225 that
  `ft4_rx::TX_TURNAROUND_BUDGET_MS` held until 2026-09-21, which
  placed the audio at 8.0 s and the frame ~200 ms late — DT ≈ +0.2 at
  the other end, inside its ±1.0 s search but spending the margin its
  own clock error needs. The cut is now this deadline, once the
  capture-time basebands and coarse sweep brought all eleven golden
  decodes inside it on a CoreS3.
- **A decode that misses it is still kept.** It is too late for this
  reply and still wanted for the screen and the choice after next;
  WSJT-X never stops decoding for a transmission, and this board only
  has a cut because it decodes on the cores the transmitter needs.
- **The other end bounds how late the audio may be.** A WSJT-X
  receiver looks at 6.048 s of the period, so a whole 5.04 s frame has
  to start within ~1.0 s of its boundary.

**Open**: the ~150 ms is WSJT-X's allowance for a PC sound card. The
CoreS3's UAC output path plus the IC-705's VOX attack is unmeasured,
and landing on DT = 0 rather than near it needs this board's own
transmission decoded on another receiver and its DT read.

### The coarse search window is a dependent variable

`stage1_inc` emits its spectrogram at pair 87, filling rows 0..173.
Block 2's last Costas symbol sits at row 162, so a lag of 11 rows still
lands it inside real data and a lag of 12 does not: **0.88 s is the
widest lag this emit point covers**, and it is `stage1_inc::max_lag_s`.

Searching wider does not fail loudly. Rows past the fill point are
present and zero, they add nothing to either sum, and the score is a
self-normalising ratio — so a lag beyond the ceiling is scored over
fewer Costas symbols and competes on equal terms with one scored over
all of them.

The window shipped at `1.0`, which `jz = round(lag / 0.08)` turned into
13 rows — ±1.04 s. Swept on the host mirror over every distinct width
the row grid allows (fixed-point, 51 capture phases per recording,
`mirror_partial_block2_policies`):

| window | lag steps | `qso3_busy` | `qso1` | `qso2` |
|---|---|---|---|---|
| ±0.80 | 21 | 331 | 88 | 87 |
| **±0.88** | **23** | **337** | **118** | **115** |
| ±0.96 | 25 | 335 | 116 | 112 |
| ±1.04 | 27 | 335 | 112 | 112 |

A maximum on all three recordings, no distinct station lost, and 15 %
off coarse sync. On a radio, 30 slots gave 9.97 ± 1.80 against 63
baseline slots at 8.54 ± 1.93 — but that is a before-and-after on an
opening band; a block-alternating harness (`MFSK_FT8_LAG_AB`) put the
window's own share at +0.50 over 24 slots, agreeing with the host
sweep. What is not a statistic: deferred candidates went from 1.27 a
slot to 0, because the ones that needed the whole slot were the ones at
the extremes of the old window.

`decode_pipeline` clamps the window to `SPEC_EMIT_MAX_LAG_S` rather
than trusting it to match — move the emit point and the ceiling moves
with it.

### Slot grid acquisition on the CoreS3

Without a disciplined clock the FT8 controller places its slot grid
from the air (#356): lock once a slot decodes enough, hold, and
re-acquire from a 25 s capture after a run of slots that do not. Three
corrections, all found on the board:

- **The capture's offset into its slot is counted.** The capture starts
  whenever acquisition is armed, part-way into a slot, and its phases are
  measured from that first sample. Uncounted, a capture ~1 s into its
  slot left the grid 1.1 s short — outside the per-slot search, ±1.0 s at the time and ±0.88 s since —
  and cost a second acquisition, about three minutes without a decode.
  What remains is the trial decodes' median bias, ~0.2 s either way.
- **Every trial phase is ranked by decode count**, rather than the first
  that decodes anything setting the grid.
- **Only full slots vote.** The partial slot after the clock anchor holds
  whatever audio remained, so its decodes say nothing about the grid;
  it is shown but neither locks nor counts as under par. This one has
  not yet been observed on hardware in the case it guards against.

## Binary footprint (Core2 reference, `xtensa-esp32-elf-size -A`)

| Region | 0.5.x BASIS | 0.6.4 Goertzel | Contents |
|---|---|---|---|
| **IRAM** (`.iram0.text` + `.iram0.vectors`) | **69 KB** | **69 KB** | esp-idf interrupt handlers, Wi-Fi/BT IRAM-resident routines |
| **DRAM** (`.dram0.data` + `.dram0.bss`) | **76 KB** | **~16 KB** | internal-RAM static data: spectrogram cache + esp-idf statics. BASIS scratch (60 KB) eliminated in 0.6.4. |
| **Flash text** (`.flash.text`) | **448 KB** | **~448 KB** | App + esp-idf code |
| **Flash rodata** (`.flash.rodata`) | **1.21 MB** | **1.21 MB** | Read-only data — **incl. the three baked WAVs (~1.08 MB)** for the offline real-audio bench |
| **Total app binary** | **~2.0 MB** | **~1.94 MB** | What `espflash flash` writes |

Subtracting the baked WAV assets (1.08 MB) and the bundled esp-idf
runtime, `mfsk-core` itself plus the M5Stack Core2 example glue
contributes roughly **150–200 KB** of flash text. The IRAM/DRAM
totals shown include esp-idf — the library proper has no IRAM
requirement and, post-Phase 1.7.7, **no internal-DRAM scratch
requirement** at all. Total per-slot working set: ~120 KB cs Box ×
1 + ~360 KB spectrogram (PSRAM) + ~12 KB BP scratch (Q11i16 since 0.6.2; was ~6 KB on Q3i8 in 0.5.x). Bare
ESP32 (no PSRAM) cannot run the spectrogram in 320 KB SRAM — PSRAM
is required for the embedded path on production-grade WAV inputs.

The **120 KB of internal DRAM freed by the BASIS drop** is exactly
what M5StickS3 Qso-mode bidirectional I2S DMA needs to allocate;
that allocation now succeeds on the first try.

## Per-protocol embedded status

| Protocol | Route to hardware | Status |
|---|---|---|
| **FT8** | `ft8::decode_block`, `fixed-point` integer pipeline | **Decoding off the air.** Six to eight stations per slot against an IC-705 on 40 m (CoreS3, 2026-08-23/24). `qso3_busy` through the same pipeline: 10 a slot with fine sync (2026-09-18, [details](#per-slot-decode-on-the-cores3-fine-sync-retries-the-key-up-bound)). The reference target; every number in [Performance benchmark](#performance-benchmark) is FT8 |
| **FST4** | generic `engine::pipeline` + `fft-extern` — **no `decode_block` port** | **Decoding off the air** on CoreS3. FST4-60 on-device: `no8_osd` 13.6 s, ≈1.95× over the ~7 s slot budget at the deadline-tight default |
| **FT4** | generic `engine::pipeline`, host f32 (`fixed-point` measured *slower* on LX7, #198) | **Decoding off the air** on CoreS3 |
| **WSPR** | host `wspr::decode` f32 via `fft-extern`, plus `wspr::ddc` | **Decoding off the air.** `slot 1 src=uac decoded 1 station(s)`. Decode lands at 82.8–90.1 s against a 110 s deadline |
| **Q65 / JT9 / JT65** | — | **Compile-clean, undriven.** #390 removed the forced `fft-rustfft`: every FFT goes through `engine::fft`, the modules carry `alloc::` imports and `num_traits::Float` instead of `std`, and JT9's `downsam9` normalises its inverse explicitly rather than assuming rustfft's unscaled convention. `alloc,<mode>,fft-extern` is in the feature matrix. What is still missing is everything after compiling: no fixed-point path, no board app, no on-device measurement |

**FST4 reached hardware without porting `decode_block`** (issue #306),
and FT4 has now done the same. `decode_block` exists because FT8's own
downsample chain needs a 192 000-point FFT; it is a way around one
specific FFT, not the definition of "runs on a chip". A corollary worth
stating because it was assumed the other way for a long time: `fst4` is
**not** a host-only feature — it type-checks clean under
`alloc,fst4,fft-extern`.

### FST4 and FT4 tuning history

Getting FST4 and FT4 inside an embedded budget took eighteen and nine
measured attempts respectively — including where the time actually
goes, which OSD levers are real, why `no8_osd` trades what it does, and
which apparent regressions were measurement artefacts. Those logs are
measurement journals rather than reference material, and they live with
the other sweeps:

- [`FST4_BENCHMARK.md` §17](../notes/FST4_BENCHMARK.md) — the eighteen
  on-device attempts, and how `decode_rung_major`'s `offsets` parameter
  came to be a caller decision rather than a decoder one.
- [`FT4_BENCHMARK.md` §50](../notes/FT4_BENCHMARK.md) — what it took to
  build at all, why the bottleneck is structural rather than
  statistical, and the candidate-budget correction.

### Live UAC bring-up

Issue [#163](https://github.com/jl1nie/mfsk-core/issues/163) — live
IC-705 hardware confirmation of the USB Audio Class capture path —
**closed 2026-08-23**: ten unbroken minutes, 125 MB, zero errors, with
WiFi associated throughout
(`embedded-poc/m5stack-cores3-app/logs/uac_stream_2026-08-23.log`).

The bring-up checklist is kept for the next time that path breaks:
[`UAC_BRINGUP_CORES3.md`](../notes/UAC_BRINGUP_CORES3.md). Read
`embedded-poc/CLAUDE.md`'s "USB host VBUS on CoreS3" and "Stacks, heaps,
and the space between them" before touching the board.

### Transmitting over the same USB cable

Receive-only until 2026-09-20; the first frame reached the radio that
day, as digital silence. What the experiment settled is a constraint
nothing in this tree had written down.

**The audio device is not the radio.** An IC-705 enumerates as three
devices behind an internal TI hub: the hub itself (`0451:2046`), an
Icom CDC composite carrying two CI-V serial pairs (`0c26:0036`), and a
**PCM2901 audio codec** (`08bb:2901`). Audio OUT is that codec's
interface 1, audio IN its interface 2.

**Only 48 kHz mono 16-bit can be opened**, and the limit is the host
controller's, not the radio's. `uac_host_device_start` refuses 2 ch /
16 bit / 48 kHz with `ESP_ERR_NOT_SUPPORTED` even though the radio
lists that format. The refusal comes from `hcd_pipe_alloc`:

```text
E HCD DWC: EP MPS (192) exceeds supported limit (128)
```

An ESP32-S3 has no HS PHY, so `otg_dfifo_depth` is 256 lines; the
default `CONFIG_USB_HOST_HW_BUFFER_BIAS_BALANCED` gives
`ptx_fifo_lines = 256/8 = 32`, and the periodic-OUT MPS limit is
`32 * 4` = **128 bytes**. The radio's OUT alt settings:

| alt | format | maxpkt | rates |
|---|---|---|---|
| 1 | 2 ch 16-bit | 192 — over the limit | 32 k / 44.1 k / 48 k |
| **2** | **1 ch 16-bit** | **96** | 32 k / 44.1 k / 48 k |
| 3 | 2 ch 8-bit | 96 | as above |
| 4 | 1 ch 8-bit | 48 | as above |

Alt 2 is the only 16-bit format that fits. FT8 transmit is mono
anyway, so nothing is lost — but the stream must then be *written* as
mono: the driver sizes its isochronous packet from the channel count
it accepted, so writing L = R into it hands the radio twice the audio
it is pacing for.

**Do not reach for `CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT`.** It
raises the limit to ~824 B and admits alt 1, and it takes the RX FIFO
from 160 lines to 34 — on the board whose receive path already lost
2.6-6.5 % of its audio to an isochronous URB budget (see
`embedded-poc/CLAUDE.md`). Spending the receive FIFO to buy a transmit
format is the wrong trade on a receiver.

Measured, twice, identically (`MFSK_CORES3_TX_PROBE=1`, amplitude 0):
632 chunks of 20 ms in **12 603 ms** against 12 640 ms of audio — 0.3 %
short, i.e. paced by the ring's own backpressure rather than by this
board's ability to synthesise. GMFSK synthesis runs from
`engine::dsp::gfsk::GfskStream`, a rotating phasor rather than `sinf`,
at ~440 us per 20 ms chunk.

**This is a probe, not a transmit path.** It runs from the enumeration
callback and blocks that task for the frame's whole 12.6 s, with
`RxConnected` queued behind it — one slot of every capture. A real
transmitter has to be driven by the QSO state machine on the slot
grid. See `m5stack-cores3-app/CLAUDE.md`'s "TX/QSO feasibility" for the
phase plan, and confirm the radio's `PTT SOURCE` is not `VOX` before
running any of it at a nonzero amplitude.

## WSPR on embedded

A second, structurally separate embedded story from everything above
— WSPR never goes through `decode_block` or `fixed-point`. It runs
the same host `wspr::decode` f32 path on-device via `fft-extern`, plus one new piece: `wspr::ddc`, a
streaming down-converter that the reference decoder's whole-slot FFT
channelizer (`wspr::baseband::decimate_to_baseband`) cannot supply on
an S3 at all — an 11.25 MiB `Complex<f32>` buffer at a
1 474 560-point FFT is neither a power of two nor within `esp-dsp`'s
8 192 ceiling. `wspr::ddc` mixes by 1500 Hz (exactly Fs/8, an
eight-entry table, no trig per sample), runs a single-stage FIR
low-pass, and keeps every 32nd sample — ~25 KB of state, independent
of slot length. Verified against the reference channelizer rather than
assumed equivalent: golden 9/9, AWGN sweep within one trial at every
SNR against 500 trials/cell, 0 phantoms either way.

Cargo features (see [Cargo features for embedded use](#cargo-features-for-embedded-use)
above for the general shape):

| Feature | What it changes | Default |
|---|---|---|
| `wspr` | WSPR protocol glue. Alone, this is a TX-only embedded-beacon build — no FFT backend required. | off |
| `wspr-ddc` | Selects the streaming down-converter over the reference whole-slot channelizer. Host keeps the exact reference; embedded has no choice — the reference cannot run there. | off (embedded's `wspr-bench` turns it on) |
| `wspr-fano-cap-fast` | Caps the Fano decoder's cycle budget at 5 000 cycles/bit (`wsprd`'s own default is 10 000, which host uses) — trades floor-SNR recall for the wall-clock a 120 s slot deadline needs. | off |
| `wspr-pass2-topn` | Ranks pass-2 candidates by refined sync and deep-processes only the top 2 (matching the dual-core split), instead of the full ladder over every survivor. | off |

Device (M5Stack CoreS3, WiFi associated throughout, dual-core):
steady state over 4 consecutive slots lands the decode at 82.8–90.1 s
against a 110 s deadline (120 s slot − a 10 s spot-upload reserve),
9/9 golden held every slot. Down-conversion, running at its real duty
cycle beside the previous slot's decode (the two overlap — a slot's
~114 s capture window is longer than one decode), costs 18.5–24.1 s.
Full measurement account — including several attempts that didn't
pan out, kept rather than deleted so they aren't retried — is
[`docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`](../notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md);
`embedded-poc/m5stack-cores3-app/src/bin/wspr_bench.rs` is the
runnable bench these numbers come from.

**Not yet verified here**: live audio capture. Everything above is
measured against a WAV-fed / synthetic baseband.
[#163](https://github.com/jl1nie/mfsk-core/issues/163), the UAC
hardware verification both lines depend on, **closed 2026-08-23** —
ten unbroken minutes of capture at 192,512 B/s and zero errors, with
WiFi associated, on the FT8 controller. `wspr_app` shares that same
`uac.rs`, so the path is proven; what has not happened is running
*this* binary against a radio. Its capture window does now open on the
UTC even-minute grid rather than wherever the USB stream came up, and
each spot carries the start time of the window it was heard in rather
than a clock read taken after the decode (#313 item 1, 2026-09-07) —
software-only, and still unverified against a radio like everything
else in this paragraph. `mfsk_app_shared::wsprnet` (wsprnet.org spot
upload, ported from WSJT-X's own `Network/wsprnet.cpp`) exists and is
off by default; its `SpotSink::Http` path is implemented but untested
against a real endpoint.

