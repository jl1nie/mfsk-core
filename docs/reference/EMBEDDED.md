# Embedded targets

`mfsk-core` is `no_std + alloc` capable: the FT8 decode path
(`mfsk_core::ft8::decode_block`) runs on chips with as little as
~150 KB of usable RAM when paired with a caller-supplied FFT
backend. This document is the reference for embedded integrators —
what the library asks of the caller, what scratch buffers are
needed, how the C ABI is shaped, and what performance to expect on
the targets we exercise today.

For host-only usage (no embedded) see [`docs/reference/LIBRARY.md`](LIBRARY.md);
for operating a receiver built on this library see
[`docs/reference/MANUAL_M5STACK_CORES3.md`](MANUAL_M5STACK_CORES3.md)
(the CoreS3, which takes audio from a radio over USB) or
[`docs/reference/MANUAL_M5STICKS3.md`](MANUAL_M5STICKS3.md)
(the StickS3 demo board).

## Architecture: how f32 and fixed-point share one codebase

The whole DSP / FEC pipeline is parameterised by **scalar traits**, so
the same source compiles to either a host-friendly f32 path or an
embedded-friendly integer path **with no duplicated code**:

- [`engine::scalar::SpecScalar`] — spectrogram / DFT-output scalar
  (`f32` on host; `Q14i16` for embedded cs storage).
- [`engine::scalar::LlrScalar`] — LLR scalar with wide-accumulator
  type (`f32` on host; **`Q11i16` with i32 wide accumulator** for
  embedded BP, since 0.6.2 — was `Q3i8` in 0.5.x. The widening
  was driven by the host f32-vs-`Q3i8` sweep on `qso3_busy.wav`
  taken pre-0.6.3 (i.e. before the 0.6.3 OSD-tightening that
  later dropped 3 CRC-luck phantoms from f32 host recall, taking
  it from 16/18 → 13/18):
  rustfft + f32 hit 16/18 while `Q3i8`'s ~0.875-LLR quantization
  step dropped that to 9/18, so the recall ceiling on Xtensa was
  the LLR resolution, not anything DSP-side. `Q11i16` removes the
  resolution bottleneck and brings host fixed-point recall up to
  f32-equivalent (full gap close); on real-silicon embedded the
  gain is only 1 entry (6/18 → 7 total) — the rest of the host gap
  is blocked by other parts of the embedded pipeline (NSTEP-half,
  coarse-sync simplifications, no `fine_refine_pass1`), not by the
  LLR scalar. BP scratch doubles from ~6 KB to ~12 KB, still inside
  the S3 / Core2 internal-DRAM budget. `Q3i8` stays in
  `engine::scalar` for the comparison path).
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

### What the fixed-point switch is wired up to today

| Component | Generic over | Fixed-point switch wired? |
|---|---|---|
| LDPC BP NMS (`fec::ldpc::bp`) | `LlrScalar` | ✅ via **`fixed-point-llr`** — a separate, opt-in feature since #349; on an LX7 the i16 loop measures 0.85× f32 |
| LLR computation (`engine::llr`) | `SpecScalar` × `LlrScalar` | ✅ via `fixed-point` (spectra) × `fixed-point-llr` (LLR type) |
| BP scratch pool (`BpScratch<P, T>`) | `LdpcParams` × `LlrScalar` | ✅ — works for FT8 LDPC(174,91) and FST4/uvpacket LDPC(240,101) |
| FT8 spectrogram + DFT (`ft8::decode_block`) | `SpecScalar` × `AudioSample` | ✅ via `fixed-point` |
| WSPR (`wspr::decode`, `wspr::ddc`) | — | ❌ — runs plain host f32 on embedded too, via `fft-extern`; never needed the integer path. See [WSPR on embedded](#wspr-on-embedded) below. |
| **FT4** | (host f32 only) | ❌ — and it does not need to be. FT4 routes through the generic `engine::pipeline`, like FST4; `fixed-point` would be a no-op on that path, and on LX7 it measured *slower* than f32 anyway (issue #198). It now **builds and decodes on hardware** — see [FT4 on embedded](#ft4-on-embedded) below. |
| **Q65 / JT9 / JT65** | (host f32 only) | ❌ — these are host-only (`fft-rustfft`, therefore `std`) and have no embedded path at all yet |

So: **the trait infrastructure is protocol-agnostic, but the only
protocol that actually flips into the integer path on the embedded
build is FT8.**

An earlier version of this paragraph said adding FT4 meant "a port of
the `decode_block` shape to FT4-specific symbol layout". That was
wrong, and issue #306 is what showed why: the generic
`engine::pipeline` *is* an embedded route — FST4 reached hardware
through it without any `decode_block` port, and FT4 has now done the
same. `decode_block` exists because FT8's own downsample chain needs a
192 000-point FFT; it is a way around one specific FFT, not the
definition of "runs on a chip".

WSPR reached embedded by a different route entirely (below), worth
noting here since the table above could otherwise be read as "only
FT8 runs on a chip": it doesn't need `decode_block`'s spectrogram/DFT
machinery or the integer pipeline, because the S3's dual-core headroom
covers WSPR's much slower cadence (a 120 s slot vs FT8's ~1.2 s
post-SlotEnd target) comfortably in plain f32.

## What we test

| Target | MCU | Backend | Status |
|---|---|---|---|
| **M5StickS3** | **ESP32-S3 (Xtensa LX7 dual-core, 240 MHz, 8 MB Octal PSRAM, ES8311 codec, ST7789P3 135×240 LCD, KEY1/KEY2)** | esp-dsp `_ae32_` asm (LX6/LX7 shared, scalar single-issue) — LX7 PIE `_aes3_` migration pending, see [`PHASE_D_PIE_SIMD.md`](../notes/PHASE_D_PIE_SIMD.md) | **Production controller** — `embedded-poc/m5stack-s3-app/` (LCD UI + QSO FSM + BLE CI-V + acoustic mic + WiFi UDP log). |
| **M5Stack Core2** | **ESP32-D0WD-V3** (Xtensa LX6, dual-core 240 MHz, single-issue f32 FPU, 16 MB flash, ~4 MB PSRAM) — confirmed by `espflash board-info`: `Chip type: esp32 (revision v3.1)` / `Features: WiFi, BT, Dual Core, 240MHz`. **Not** an ESP32-S2 (LX7, single-core, no BT) or S3. | esp-dsp ASM (`dsps_dotprod_s16_ae32`, `dsps_fft2r_*`) | **Production app (`wav_sim` only)** — `embedded-poc/m5stack-core2-app/` runs the same `decode_block` against the baked `wav_sim` audio loop on LX6 to cross-validate the `mfsk-app-shared` API. Classic ESP32 has no USB peripheral, so live mic / speaker / USB-Host paths are not on the table for this board — Core2's role is the second-board LX6 verifier for the shared QSO FSM. (The original standalone Core2 compute bench `embedded-poc/m5stack-core2/` was retired in #61 Phase 3 once this app crate covered the same wav_sim path in production-app shape.) |
| ESP32-S3 compute bench | Xtensa LX7 | esp-dsp ASM | **Timing-regression bench** — `embedded-poc/m5stack-s3/`, drives `decode_block` against canned WAV inputs for per-stage timing sweeps. Not for end users. |
| **M5Stack CoreS3** | ESP32-S3 LX7 + AXP2101 PMIC + AW9523B I/O expander (USB host VBUS needs port1 bit7 `BOOST_EN` + port0 bit5 `USB_OTG_EN` + port0 bit1 `BUS_OUT_EN`, all three — see `embedded-poc/CLAUDE.md` "USB host VBUS on CoreS3") | esp-dsp `_ae32_` asm (same Phase D D1 migration applies) | **Main UAC controller target** (Phase B-Core, 2026-05-17 pivot) — `embedded-poc/m5stack-cores3-app/`. Phase 0-Core (bringup) + Phase 1-Core (AW9523B BUS_OUT_EN + UAC host) shipped in commit `1a93c92`. M5StickS3 cannot do USB-OTG host (no VBUS source circuit), so it was repositioned as the **demo / acoustic-fallback** board and the live USB Audio Class path to IC-705 lands on CoreS3 instead. See `docs/notes/ROADMAP.md` Phase B-Core. |

### Other targets — what's verified vs aspirational

The `fft-extern` contract is *designed* to be target-portable, and
`mfsk-ffi-ft8` cross-builds cleanly to several non-Xtensa MCUs:

| Target | `cargo build` clean | FFT shim shipped | Hardware-tested |
|---|---|---|---|
| `xtensa-esp32-espidf` | ✅ | ✅ esp-dsp (Core2) | ✅ qso1/2/3 sweep |
| `xtensa-esp32s3-espidf` | ✅ | ✅ esp-dsp (S3 bench + S3-app + CoreS3-app bring-up, Phase B-Core) | ✅ qso1/2/3 sweep |
| `thumbv8m.main-none-eabihf` (RP2350 Cortex-M33) | ✅ | ❌ candidates: CMSIS-DSP via pico-sdk-rs | ❌ |
| `riscv32imac-unknown-none-elf` (RP2350 Hazard3) | ✅ | ❌ no DSP library; `microfft` for FFT | ❌ |
| `thumbv7em-none-eabihf` (Cortex-M4F / M7) | not tried | ❌ candidates: CMSIS-DSP `arm_*_q15` | ❌ |
| `thumbv6m-none-eabi` (Cortex-M0+ / RP2040) | not tried | ❌ scalar Rust only (no DSP unit) | ❌ |

**ESP32 / ESP32-S3** (Xtensa LX6 / LX7) are the only targets we
exercise end-to-end with real audio. For everything else, the
library *can* build (try
`cargo build -p mfsk-ffi-ft8 --release --no-default-features
--features embedded-fixed-point,embedded-runtime --target <T>`),
but you'll need to supply the FFT extern Rust symbol yourself.
Concrete RP2040 / RP2350 / Cortex-M shims are tracked as future
work.

`embedded-poc/embedded-shared/src/esp_dsp_fft.rs` is the worked
example to copy from.

## Cargo features for embedded use

Default features include `std`, `parallel`, and `fft-rustfft` — turn
those off and pick the embedded baseline:

```toml
[dependencies]
mfsk-core = { version = "0.8", default-features = false, features = [
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
| `fft-extern` | FFT backend via `mfsk_core_make_default_fft_planner` extern fn (and the i16 variant `_planner16`). | Any embedded target. |
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
pub extern "Rust" fn mfsk_core_make_default_fft_planner16()
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

### Removed: i16 × Q15 dot product extern (0.8.0, issue #162)

A separate `mfsk_core_dot_q15_i32` extern symbol used to be required
for the per-symbol DFT (the BASIS path). Since 0.6.4 (Phase
1.7.7-Stick) that extern was already unused by the decoder — the
per-symbol DFT ran through the in-tree Goertzel recursion (see next
section) — and 0.8.0 removed the symbol, the `mfsk_core::core::dotprod`
module, and the BASIS fill path entirely. New integrations don't need
to implement anything here at all.

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

## Using from C / C++ / non-Rust ESP-IDF projects (`mfsk-ffi-ft8`)

[`mfsk-ffi-ft8`](https://github.com/jl1nie/mfsk-core/tree/main/mfsk-ffi-ft8)
exposes a tiny C ABI for the FT8 block decoder. It is the
recommended way to call the embedded FT8 decoder from a non-Rust
ESP-IDF (or RP2040 / Cortex-M) project.

The crate is `no_std + alloc` under its `embedded-fixed-point`
feature so the resulting `libmfsk_ft8.a` doesn't carry Rust's `std`
runtime — drop-in linkable from C without the toolchain weirdness
that would come from mixing two libc layers.

**Verified end-to-end on ESP32 Core2** (originally on the
`embedded-poc/m5stack-core2/` standalone compute bench — a
development-only timing harness, retired in #61 Phase 3 (0.6.3)
once `embedded-poc/m5stack-core2-app/` covered the same wav_sim
path in production-app shape): a separate `ffi_smoke_one` path called
`mfsk_ft8_decode_i16` (C ABI) on the same baked WAVs as the
direct-Rust `decode_one` path and got identical recall — qso1
(3 / 3), qso2 (5 / 5), **qso3 busy band (7 / 7)**. With
caller-managed BASIS scratch in internal RAM the FFI path landed
~2.6 × faster than the same FFI call with internal heap allocation
(qso3 3.74 s vs 9.57 s). Post-Goertzel (0.6.4+) the same recall
holds with **no scratch arguments needed at all**.

### API at a glance

cbindgen-generated header — `mfsk-ffi-ft8/include/mfsk_ft8.h`,
regenerated on every build. The full surface is:

```c
typedef struct MfskResult {
    char     text[40];   // NUL-terminated unpacked message
    float    freq_hz;    // carrier
    float    dt_sec;     // time offset relative to slot start
    float    snr_db;     // calibrated against JTDX absolute via
                         // xsnr2_db_simple (within ±3 dB on real silicon)
    uint32_t hard_errors;
    uint8_t  pass;       // staircase stage (0=fast Bp, 1=full Bp,…)
} MfskResult;

typedef struct MfskResultList {
    MfskResult *items;
    size_t      len;
    size_t      _capacity;  // private
} MfskResultList;

// Opaque decode-tuning handle — construct with mfsk_ft8_options_new,
// release with mfsk_ft8_options_free. NULL is always a valid options
// argument (uses this crate's built-in default: 200-3000 Hz, sync_min
// 1.0, max_cand 30, MFSK_DECODE_DEPTH_BP_ALL_OSD).
typedef struct MfskDecodeOptions MfskDecodeOptions;

MfskDecodeOptions *mfsk_ft8_options_new(
    float freq_min_hz, float freq_max_hz,     // typical 200, 3000
    float sync_min, int max_cand,             // typical 1.0, 30
    MfskDecodeDepth depth);                   // 1=BpAll, 2=BpAllOsd
void mfsk_ft8_options_free(MfskDecodeOptions *opts);

// PRIMARY embedded entry. Needs no caller-managed scratch — the
// decoder fills its per-symbol DFT via the in-tree Goertzel
// recursion (zero internal-DRAM scratch). Before 0.8.0 (issue #162)
// this function also took `basis_re`/`basis_im` scratch pointers for
// a since-removed BASIS fill path; before 0.8.0 (issue #205) it also
// took the five tuning knobs positionally instead of via `options`,
// and host builds exposed a separately-named `mfsk_ft8_decode_i16_alloc`
// — C callers built against pre-0.8.0 headers must update both.
MfskStatus mfsk_ft8_decode_i16(
    const int16_t *audio, size_t n_samples,   // 12 kHz, mono, ≥168 000
    const MfskDecodeOptions *options,         // NULL = built-in default
    MfskResultList *out);                     // populated by callee

void mfsk_ft8_result_list_free(MfskResultList *list);
```

### Calling `mfsk_ft8_decode_i16`

No scratch buffers to manage — just call it:

```c
#include "mfsk_ft8.h"

MfskDecodeOptions *options = mfsk_ft8_options_new(
    200.0f, 3000.0f, 1.0f, 30, MFSK_DECODE_DEPTH_BP_ALL);

MfskResultList results = {0};
MfskStatus st = mfsk_ft8_decode_i16(audio, n_samples, options, &results);
// ... use results ...
mfsk_ft8_result_list_free(&results);
mfsk_ft8_options_free(options);
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
MfskStatus mfsk_ft8_stream_push_i16(MfskFt8Stream *,
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
static MfskDecodeOptions *g_options;
static int16_t g_slot[180000];          // 360 KB; OK in PSRAM

void rx_init(void) {
    g_stream = mfsk_ft8_stream_new(/*src*/16000, /*cap*/180000);
    g_options = mfsk_ft8_options_new(200.0f, 3000.0f, 1.0f, 30,
                                      MFSK_DECODE_DEPTH_BP_ALL);
}

// Capture task: I2S DMA callback
void on_i2s_chunk(const int16_t *samples, size_t n) {
    mfsk_ft8_stream_push_i16(g_stream, samples, n);
}

// Decode task: fires every 15 s on UTC slot boundary
void on_slot_boundary(void) {
    if (mfsk_ft8_stream_buffered_samples(g_stream) < 168000) return;
    size_t n = mfsk_ft8_stream_peek_latest(g_stream, g_slot, 180000);

    MfskResultList results = {0};
    mfsk_ft8_decode_i16(g_slot, n,        // n, not 180000 — peek may
                                          // return fewer if the ring
                                          // isn't yet full.
                        g_options, &results);
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
/tmp/mfsk_smoke embedded-poc/assets/qso3_busy.wav
```

#### Embedded (Xtensa ESP32, `libmfsk_ft8.a` for ESP-IDF link)

```sh
source ~/export-esp.sh                     # Xtensa toolchain
RUSTFLAGS="-C panic=abort" \
cargo build -p mfsk-ffi-ft8 --release \
    --no-default-features \
    --features embedded-fixed-point,embedded-runtime \
    --target xtensa-esp32-espidf            # or -esp32s3-espidf
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
| `host` | ✓ | Host build — pulls `mfsk-core/std + ft8 + fft-rustfft`. Exports `mfsk_ft8_decode_i16` backed by the host-native f32 path. Before 0.8.0 (issue #205) this feature exported a separately-named `mfsk_ft8_decode_i16_alloc`; the symbol is now unified with the embedded build's — same name, host-native backend. |
| `embedded-fixed-point` | — | `no_std + alloc`. Pulls `mfsk-core/fft-extern + fixed-point` (which implies `nstep-half`). Exports the same `mfsk_ft8_decode_i16` symbol, backed by the fixed-point path. The linker must resolve `mfsk_core_make_default_fft_planner` + `_planner16` (typically via a small Rust shim that bridges esp-dsp). |
| `embedded-runtime` | — | Provides default `#[panic_handler]` (calls libc `abort`) + `#[global_allocator]` (libc `malloc`/`free`). Needed for a self-contained `staticlib`; turn off when stacking another Rust runtime in the same image. |

### Linking it into an ESP-IDF (CMake) project

```text
your-app/                          # esp-idf project root
├── main/main.c                    # calls mfsk_ft8_decode_i16(...)
├── components/mfsk_ft8/
│   ├── CMakeLists.txt             # IMPORTED static-lib component
│   ├── include/mfsk_ft8.h         # from mfsk-ffi-ft8 build
│   └── lib/libmfsk_ft8.a          # from mfsk-ffi-ft8 build
└── shim/                          # tiny Rust crate (esp-dsp bridges)
    ├── Cargo.toml                 # depends on mfsk-ffi-ft8
    ├── .cargo/config.toml         # target = xtensa-esp32-espidf, panic=abort
    └── src/lib.rs                 # provides mfsk_core_make_default_fft_planner[16]
```

The `shim/` Rust crate is needed because mfsk-core's FFT-extern
contract uses `extern "Rust"` symbols (different ABI from
`extern "C"`), which a pure-C compilation unit can't satisfy. The
shim is ~50 lines of Rust + a vendored copy of
`embedded-poc/embedded-shared/src/esp_dsp_fft.rs`.

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

#### Why we don't widen PASS1 / enable OSD on the embedded path

Tested against the WSJT-X reference busy band on real S3 LX7
silicon (`logs/s3_pass100_max30_2026-05-04.log`):

| config | qso3 post-SlotEnd | qso3 recall | total recall |
|---|---:|---:|---:|
| Bp/30/15 (ship)  | **~1.2 s** | 7/18 | 14/22 (or 15 with phantom) |
| Bp/100/30        | **~1.6 s** | 7/18 (unchanged) | +1 (qso1 OH3NIV only) |
| DecodeDepth::FULL/200/100 (host estimate) | ~7 s | 7/18 (+1 on qso3 N1JFU) | 16/22 |

Two non-obvious findings drove the decision to stay at `PASS1=30 /
max_cand=15`:

1. **qso3 busy band recall is bounded by coarse_sync rank, not BP /
   OSD effort.** Widening PASS1 from 30 → 100 + max_cand 15 → 30
   recovers no qso3 calls — the missed signals are below coarse_sync
   rank 100 entirely. They need iterative subtraction (the WSJT-X
   wide-band path's hallmark) which `decode_block` doesn't implement.
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
the 2 s budget cleanly. Pushing further requires either (a) porting
iterative subtraction to the embedded path (open question on cost
— see `docs/notes/ROADMAP.md` "Embedded fine_refine attempt postmortem")
or (b) accepting late-arrival "spotter mode" decodes that land too
late for QSO turnaround.

Per-stage breakdown on `qso3_busy.wav`:

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

## WSPR on embedded

A second, structurally separate embedded story from everything above
— WSPR never goes through `decode_block`, `fixed-point`, or
`mfsk-ffi-ft8`'s C ABI. It runs the same host `wspr::decode` f32 path
on-device via `fft-extern`, plus one new piece: `wspr::ddc`, a
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
*this* binary against a radio. `mfsk_app_shared::wsprnet` (wsprnet.org spot
upload, ported from WSJT-X's own `Network/wsprnet.cpp`) exists and is
off by default; its `SpotSink::Http` path is implemented but untested
against a real endpoint.

## FST4 on embedded

A third, distinct embedded story — not started before issue #306
asked whether it was even feasible, and not predictable from either
of the two above. FT8's integer `decode_block`/`fixed-point` path
doesn't apply (FST4 routes through the generic f32
`engine::pipeline`, same as WSPR); WSPR's own measured result doesn't
transfer either, since its dominant cost (issue #260: Fano-sequential
search burning a full node budget per failing candidate, ~51% of a
scan) is a failure mode specific to WSPR's convolutional decoder —
FST4's LDPC(240,101) + bounded belief-propagation + OSD-fallback path
fails cheaply by construction, no analogous budget blowout expected
a priori. Untested claim, not a measured one, as of this writing.

`fst4` (the Cargo feature) turned out not to need `fft-rustfft`/`std`
at all — a stricter-than-necessary gate left over from when it was
grouped with jt9/jt65/q65 under "heavy modes, out of scope of the
embedded port" (those three do call `rustfft::` directly; FST4 never
did, it just inherited the same feature bound). `mfsk-core/src/fst4/`
had two latent `no_std` gaps this had never exercised — `encode.rs`
and `baseline.rs` both used bare `Vec` relying on `std`'s prelude
without an explicit `alloc::vec::Vec` import — fixed alongside
loosening the feature.

Same wideband-stage problem as WSPR, same fix shape: FST4-60A's
`build_fft_cache` runs one `fft1_size = 746_496`-point forward FFT
over the 60 s slot, past `esp-dsp`'s 8 192 ceiling exactly like
WSPR's `decimate_to_baseband`. `engine::pipeline::decode_frame`
already had a `precomputed_fft` parameter for this (mirroring FT8's
own `decode_frame_inner`) — unlike WSPR, no new streaming front end
was needed, just feeding a host-baked cache through an existing seam.
The bake step (`fst4_bake_golden_precomputed` in
`mfsk-core/tests/fst4_wsjtx_samples.rs`) round-trips both baked files
back through `decode_frame` on the host and asserts the result
matches `DecodeRequest`'s own fresh-computed path exactly.

`embedded-poc/m5stack-cores3-app/src/bin/fst4_bench.rs` (thin shim
over `embedded_shared::apps::fst4_bench`) is a decoder-only bench —
deliberately narrower in scope than `wspr_bench` was even at its
first commit: single-shot, single-core, no PSRAM/SRAM arms, no WiFi.
Issue #306 asked for exactly that much ("I'm not suggesting a full
embedded FST4 port... A decoder-only benchmark would be enough"), and
`wspr_bench`'s own multi-arm shape only exists because real
measurement rounds kept finding things worth splitting out — building
that ahead of a first FST4 number would be guessing.

**First real-device attempt (2026-08-16): blocked, cause identified,
not a memory or compute-budget question after all.**

Two infrastructure bugs surfaced and got fixed on the way to a real
measurement, neither specific to FST4:

1. **`espflash` writes a 4 MiB flash-size into the app image header
   when `--flash-size` isn't passed**, regardless of what it
   correctly auto-detects from the connected chip for its own log
   output. Every app ever flashed to this CoreS3 before `fst4-bench`
   happened to fit under 4 MiB total (factory + littlefs), so this
   was silently wrong the whole time and never mattered — the
   9 MiB factory partition this bench needed was the first thing to
   expose it (`E (NN) boot: Failed to verify partition table` /
   `flash_parts: partition N invalid ... exceeds flash chip size
   0x400000`, persisting across a full `espflash erase-flash`, since
   the bad header ships with every rebuild). Fixed in
   `embedded-poc/scripts/flash-monitor.sh` — takes an optional 6th
   `FLASH_SIZE` argument (`16mb` for this board) that becomes an
   explicit `--flash-size` flag; see the script's own comment for the
   diagnosis. Old callers that don't pass it keep the previous
   (buggy-but-so-far-harmless) behaviour.
2. `m5stack-cores3-app/partitions.csv`'s `factory` partition grew
   from 3 MiB to 9 MiB for the ~7.7 MiB app image the baked assets
   produce — shared across every `[[bin]]` in that crate, so
   re-flashing after this change resets the littlefs-stored settings.

With both fixed, the app boots and loads its baked assets cleanly
(1 008 ms for 1.44 MiB audio + 5.7 MiB `fft_cache`, PSRAM headroom
fine — 893 KiB free post-load against the ~14.3 MiB combined
Vec+baked-bytes peak the module doc flagged as the open risk; that
risk did not materialize). It then panics on the very first FFT call,
inside `coarse_sync`'s spectrogram stage, before either the PSRAM
question or a wall-clock number could be reached:

```text
esp-dsp FFT requires power-of-2 length ≥ 4 (got 7776)
```

`7776 = NSPS(3888) × NFFT_PER_SYMBOL_FACTOR(2)` —
`engine::sync::SyncDims::of::<Fst4s60>()`'s coarse-sync spectrogram
FFT length, and `7776 = 2⁵ × 3⁵` is not a power of two. This is the
exact same shape as FT8's own spectrogram FFT (`NSPS(1920) × 2 =
3840`, also not a power of two) — which is why FT8 has a hand-rolled
`MixedRadix3840Fft` in `embedded-shared::esp_dsp_fft` rather than
using the ESP-DSP backend's power-of-two-only radix-2 kernel
directly. FST4 has no equivalent, and would need a second one for
7776 specifically (and likely a third: `downsample_cached`'s inverse
FFT at `fft2_size = 6912 = 2⁸ × 27`, also non-power-of-two, has not
been reached yet — `coarse_sync` panics first).

**So the answer to issue #306's actual question is not yet "does it
fit the slot" — it's "the embedded FFT backend needs a new
non-power-of-two kernel before FST4's candidate search can run on
device at all".** Unlike WSPR's `decimate_to_baseband`, this isn't a
size problem `esp-dsp` was never going to solve (WSPR's 1 474 560-pt
whole-slot FFT stays baked host-side by design) — 7776 points is
trivially small for a per-symbol FFT, it's specifically the *radix*
ESP-DSP's asm kernels don't support.

**Second attempt, same day: route around the FFT sites entirely
instead of building #307's kernel first.** `coarse_sync` and
`downsample_cached`'s FFTs are only in the *sync/refine* stage, not
LLR/BP/OSD itself — so a candidate already refined on a host (same
`(cd0, freq_hz, i0, score)` shape `process_candidate_basic_impl`'s
existing `precomputed_refine` parameter accepts) lets the device skip
both without waiting on #307. `mfsk-core/tests/
fst4_wsjtx_samples.rs::fst4_bake_golden_refined_candidates` bakes
every real (post-dedup) candidate this way, and a new
`engine::pipeline::process_candidate_precomputed` (`internal-testing`-
gated, mirrors `process_candidate_basic`) exposes the seam externally.
Flashed clean — then panicked again, same assertion, `got 36` this
time: `GenericPipelineProtocol::snr_db`'s FST4 override
(`fst4_snr_db`) turned out to call `downsample_cached` a *third*,
independent time (`fst4_raw_cs` needs the non-normalised spectrum,
which the already-normalised baked `cd0` can't substitute for) —
closed with a new `skip_snr` parameter (`NAN` in `DecodeResult::snr_db`
instead of calling `P::snr_db`; this bench measures wall-clock, not
SNR accuracy). Rebuilt, reflashed — **panicked a third time, same
assertion, still `got 36`**: `engine::llr::symbol_spectra` (the actual
first step of LLR extraction, called for every candidate regardless of
`precomputed_refine`) plans its own `ds_spb = NSPS/NDOWN`-point FFT
(36 for FST4-60A) that neither fix touched.

**Third attempt: 36 is small enough not to need a real FFT at all.**
`ds_spb` is 36-42 across all five FST4 submodes (`REFINE_STEPS`'s own
doc comment already noted this range). A plain O(N²) direct DFT —
the textbook definition, not an approximation of one, since every FFT
computes the identical sum by a faster route — costs at most 42² =
1 764 complex multiply-adds per call, negligible next to the LLR/BP/OSD
work it feeds. Added as `embedded-shared::esp_dsp_fft::DirectDft`,
wired into `EspDspPlanner::plan_forward` for any non-power-of-2 length
up to `DIRECT_DFT_MAX_LEN = 64`; verified against `numpy.fft` on host
(same unnormalised forward convention `rustfft` uses) at N=36 and
N=42, ~1e-6 max error — f32-level precision, no sign/indexing bug.

With all three FFT sites closed (candidate refine baked, SNR skipped,
symbol-spectra FFT replaced), the on-device path has zero FFT calls
left. The fourth flash attempt (after adding `DirectDft`) stalled
mid-write — `espflash`'s progress bar stopped advancing partway
through a full post-erase write, and the board stopped responding to
`espflash board-info` afterward. Diagnosed as an unrelated USB/serial
hang (device node stayed present, no USB detach in dmesg), not a code
regression — confirmed once the user power-cycled the board and it
came back immediately, no further recovery steps needed.

**Fifth attempt, PSRAM this time.** Rebuilt, reflashed clean, assets
loaded fine — then `memory allocation of 524288 bytes failed` inside
the candidate loop. Both baked assets were still being shipped:
`fft_cache` (5.7 MiB, from the *first* attempt's fix, `fst4_bake_
golden_precomputed`) plus the 2.16 MiB of baked `cd0` buffers left
only 87 KiB PSRAM free post-load, and the loop's own transient BP/OSD
allocations didn't fit. `fft_cache` turned out to be genuinely dead
weight on this path — `precomputed_refine` skips the one call that
would read it (`downsample_cached`) and `skip_snr = true` skips the
other (`fst4_raw_cs`, inside `P::snr_db`) — so it was never
dereferenced, only occupying space. Dropped it from the bench
entirely (`run_bench` no longer takes it); PSRAM free post-load went
87 KiB → 5.97 MiB.

**Sixth attempt: a complete run, first ever FST4-on-embedded wall-clock
number.** CoreS3 @ 240 MHz / `opt-level = 3`, single core, no FFT
calls anywhere in the path:

| | value |
|---|---:|
| candidates | 41 (post-dedup, of 50 raw `coarse_sync`) |
| decodes | 2/2 — `CQ N5TM EL29`, `CQ K9KFR EN71` (matches host exactly) |
| candidate-loop wall-clock | **89.691 s** |
| host `decode_loop` for the same 41 candidates | 51.9 ms (`MFSK_TRACE_STAGE_FST4=1`, this box) |
| device/host ratio | **≈1728×** |
| vs. FST4-60's ~7 s post-slot margin | **≈13× over** |

**The ratio is the finding, not just the seconds.** It lands close to
WSPR's own *first*, wholly-unoptimized measurement (issue #260:
1214.3 s against a 709.5 ms host baseline, ≈1712× — before
`minsync2`, `opt-level=3`, or the 160→240 MHz clock fix, the three
biggest levers that investigation eventually found). This FST4 bench's
build already carries two of those three (`opt-level=3` and the clock
fix are both `m5stack-cores3-app`-wide, not per-bin) — so it starts
from a more favourable position than WSPR's raw first number did, and
still landed in the same territory. Earlier text in this section (and
in `embedded-shared::apps::fst4_bench`'s own module doc) repeated the
premise that FST4's bounded LDPC/BP/OSD "fails cheaply by
construction" and so shouldn't need WSPR's kind of dedicated tuning
pass, explicitly flagged as *"an untested claim, not a measured one."*
It is now tested, and at this unoptimized state it does not hold —
something in FST4's LLR/BP/OSD path costs about as much per real-world
candidate as WSPR's Fano-sequential search did before WSPR's own
four-round optimization campaign. Two unmeasured suspects, not
diagnosed further here: the `LLR_NSYM_MAX = 8` staircase rung (`4⁸ =
65536` tone-combination hypotheses per group — `fst4::decode`'s own
lazy-staircase code calls this "128-256× FT8/FT4's own deepest rung"),
and that FST4 here runs the plain f32 generic pipeline with *zero*
embedded-specific optimization work done on it, unlike FT8's dedicated
fixed-point `decode_block` or WSPR's now-tuned `decode_scan`.

Peak stack usage, measured in passing: `BENCH_STACK`'s 96 KiB guess
left 95 064 B untouched — only ~3.2 KiB actually used, a wide margin
unlike `wspr_bench`'s own stack history.

**Seventh and eighth attempts: where the 89.7 s actually goes.** Added
per-candidate wall-clock logging (dumped after the timed loop, not
during it, so UART writes don't contaminate the numbers) and a
build-time `MFSK_FST4_BENCH_DEPTH=bp_only` switch
(`DecodeDepth::BP_ONLY` vs. the default `::FULL` — the two differ only
in `osd: bool` for FST4, `LlrEffort` doesn't apply to it — same shape
as WSPR's own issue #260 controlled experiment, `confirmed = None`
short-circuiting before `osd_decode`).

The 89.4 s is not spread across all 41 candidates. 94% of it is 6
candidates, every one a failure:

| tier | count | each | total | decoded |
|---|---:|---:|---:|---|
| nsync-gate fails | 33 | 50-177 ms | ~6 s | no |
| **the tail** | **6** | **13.8-14.2 s** | **84.2 s (94%)** | **no** |
| real signals | 2 | 54-58 ms | 0.1 s | **yes** |

Both real decodes are cheap. The entire cost is 6 candidates the
decoder ultimately rejects — the same shape WSPR's issue #260 found
("OSD ran 896 times and succeeded 0 times... the cost is not where any
of us was looking").

Re-running those same 41 candidates with OSD off splits that tail:

| | total | tail avg | share |
|---|---:|---:|---:|
| `FULL` (OSD on) | 89.411 s | 14.04 s | — |
| `BP_ONLY` (OSD off) | 51.755 s | 7.76 s | — |
| OSD's contribution | 37.66 s | 6.28 s | **42%** |

Both decodes still succeed with OSD off — neither real signal on this
file ever needed it. So OSD is a real cost, but not the majority one:
**plain BP/LLR alone is 51.8 s, already ≈7.4× over the ~7 s budget by
itself.** Cutting OSD would help (42% of the total) but wouldn't be
sufficient on its own.

**Ninth attempt: the `LLR_NSYM_MAX = 8` suspect, confirmed and fully
localized.** `MFSK_FST4_BENCH_DEPTH=llr_probe` calls `symbol_spectra` +
each `compute_llr_*` stage directly with no BP/OSD at all, timing
every stage for all 41 candidates (this mode skips the real pipeline's
nsync-gate early exit on purpose, to compare every stage's cost on
equal footing rather than only the survivors):

| stage | total (41 candidates) | share |
|---|---:|---:|
| `symbol_spectra` | 1.39 s | 0.5% |
| nsym=1 (`compute_llr_fast`) | 0.09 s | <0.1% |
| nsym=2 | 0.14 s | <0.1% |
| nsym=4 (`LLR_NSYM_MID`) | 1.10 s | 0.4% |
| **nsym=8 (`LLR_NSYM_MAX`)** | **301.2 s** | **99.1%** |

`nsym=8` alone costs **~7.347 s per candidate**, uniform to within
0.01% regardless of which candidate — a pure function of the
computation's size (`4⁸ = 65536` tone-combination hypotheses per
symbol group), not data-dependent. That single stage accounts for
essentially all of the `BP_ONLY` run's ~7.76 s/candidate tail
(0.034+0.002+0.003+0.027+7.347 ≈ 7.41 s of it; the ~0.35 s remainder
is presumably `decode_soft_pooled`'s own BP iterations, not separately
measured here).

One thing this *doesn't* change: `SYNC_Q_MIN = 16` is already FST4's
own `minsync2` equivalent — WSJT-X's own pre-ladder gate
(`get_fst4_bitmetrics.f90`), faithfully ported (issue #197) — and it's
what keeps 33 of 41 candidates from ever reaching this stage in the
real pipeline. The 8 that do clear it are the same population a real
`jt9` has to run this same computation on. Unlike WSPR's `minsync2`
gap, there's no missing cheap-reject lever here to find — the cost is
intrinsic to the candidates that legitimately warrant deep decoding,
not a filtering gap this codebase left unfilled.

So the remaining levers are inside `compute_llr_partial` itself
(algorithmic restructuring, fixed-point, SIMD/PIE), accepting a recall
trade-off by skipping `nsym=8` on embedded the way FT8's ship config
skips OSD entirely, or parallelism (dual-core).

**Tenth attempt: one of those levers, taken.** Disassembling
`fill_bmet_for_nsym`'s compiled Xtensa output
(`xtensa-esp32s3-elf-objdump` against the bench's own ELF) found that
its hot loop's two `f32::max()` calls each compiled to a real
`callx8 fmaxf` — Xtensa's FPU has no native float max/min instruction,
so LLVM can't lower `max`'s IEEE-754 NaN-propagation semantics to a
single compare. At `nsym=8` that's ~42 M subroutine calls per
candidate (2 calls × 65536 elements × 16 bit positions), not 42 M
single-cycle compares.

The two operands at that call site (`v_for_one`/`v_for_zero`) are
always either `sqrt(re²+im²)` (finite, ≥ 0) or the literal
`f32::NEG_INFINITY` — never NaN — so `max`'s NaN handling is provably
dead weight on this specific loop. Replaced with a plain `>` compare
(bit-identical for non-NaN inputs — confirmed against every existing
golden/AWGN FT8/FT4/FST4 test, all byte-for-byte unchanged).
Re-disassembling confirmed the fix: the hot loop compiles to Xtensa's
native `ule.s` compare + `bt`/`bf` branch now, zero `callx8` in that
loop (one `fmaxf` call remains elsewhere in the function, for the
once-per-bit-position normalisation step — 16 calls/group instead of
2×65536×16, left alone as negligible).

Measured, same CoreS3, same golden, same 41 candidates:

| | before | after | speedup |
|---|---:|---:|---:|
| `nsym=8` (41-candidate `llr_probe` total) | 301.2 s | 178.1 s | 1.69× |
| `nsym=4` (same) | 1.10 s | 0.61 s | 1.80× |
| **`FULL` candidate loop (real 8-candidate population)** | **89.411 s** | **71.312 s** | **1.25×** |

Still 2/2 real decodes, identical to every prior run. A genuine,
zero-risk, two-line win — but well short of the ~1.7× its own `nsym=8`
isolation would suggest (the loop's other per-element cost — indexing,
the branchless-select setup, `s2` traffic — was already there and
untouched, so removing just the call recovers less than the call's
own share of one iteration), and far short of the ~13× gap this
session's first measurement found.

**Eleventh attempt: OSD's own allocator traffic — the fmaxf fix's
same shape, applied to the now-larger half.** Re-tallying the
`FULL`/`BP_ONLY` split after the LLR fix put OSD at 53% of the total
(up from 42% — the LLR fix shrank the denominator, OSD itself was
untouched). `osd_decode_generic::<Ldpc240_101Params>` — FST4's OSD
entry point, generic code shared with FT8's `Ldpc174_91Params` and
MSK144's `Ldpc128_90Params` — is an order-3 combinatorial search:
`try_candidate` runs `C(101,3) ≈ 166 650` times per call for FST4, and
its closure allocated two fresh heap `Vec<u8>` (240 B + 101 B) on
*every* call, freed again immediately on the overwhelmingly common
path where CRC verification fails. Disassembly confirmed it: 9
`__rust_alloc_zeroed` and 38 `__rust_dealloc` call sites in one
4.2 KiB function.

`P::N`/`P::K` are compile-time consts on `LdpcParams`, but `[u8;
P::N]` isn't expressible in a function generic over `P` on stable
Rust — confirmed directly (`error: generic parameters may not be used
in const operations`; `generic_const_exprs` remains nightly-only).
Used the same "fixed max bound + runtime-length prefix slice" idiom
`engine::llr`'s `MAX_NSYM`/`MAX_IBMAX_PLUS_1` already established for
the identical problem: a `[u8; 256]` stack buffer (≥ 240/174/128, the
three protocols' `N`) sliced to `[..n]`, reused across all ~166 650
calls instead of allocated fresh each time. Bit-identical on host —
confirmed against all three protocols' goldens (FT8 full-parity 8/8,
FT8 ship-config, MSK144, FST4-60), not just FST4's alone.
Re-disassembling: `__rust_alloc_zeroed` 9 → 1, `__rust_dealloc` 38 → 1
(the one remaining call of each is `osd_setup_generic_packed`'s
one-time setup and the final `OsdResult` construction, not
per-candidate).

| | before | after | speedup |
|---|---:|---:|---:|
| **`FULL` candidate loop** | **71.312 s** | **67.789 s** | **1.052×** |

Real, but smaller than the alloc-call-count reduction alone would
suggest — the same shape as the LLR fix: removing the allocator calls
recovers only their own share of a `try_candidate` iteration, and the
surrounding O(n) XOR/permute/weighted-distance work (n=240) was
already there and untouched.

Cumulative at this point: 89.411 s → 67.789 s, 1.319×, ≈9.7× over the
~7 s budget (down from ≈13×).

**Twelfth attempt: one lever on each side — VK3NV's `bit_sel` block
reduction on the LLR side, and reordering OSD's `verify` gate ahead of
its own O(n) scatter.**

*LLR side.* VK3NV (issue #306) pointed out that `fill_bmet_for_nsym`'s
inner loop still had a shift, a mask and two branchless selects per
element even after the `fmaxf` fix — and that for a fixed `bit_sel =
b`, `(i >> b) & 1` isn't data-dependent at all: it's the known
periodic pattern `2^b` zeros then `2^b` ones, repeating. `nt =
ntones^nsym` is always a power of 2 (every protocol's `ntones` is), so
`block = 2^bit_sel` always divides `nt` evenly — the loop can walk
`s2` in `2^bit_sel`-sized contiguous blocks instead, alternating which
accumulator (`mo`/`mz`) each block feeds, with a single compare per
element and no per-element predicate machinery at all. Same
element-visit count as before (`nt` per `bit_sel`), only the
per-element overhead collapses. Bit-identical on host across FT8, FT4,
FST4 and MSK144's shared code path — this loop is used by all three.

*OSD side.* `try_and_update`'s scatter (`c[perm[col]] = cp[col]`,
`O(n)=240`) ran on *every* one of the ~166 650 calls, before the
`verify` (CRC-style) gate that rejects nearly all of them — the
scatter's whole purpose was producing `decoded = c[..k]` for `verify`
to check, but building the full `n`-length codeword to get the first
`k` bits is unnecessary: an inverse permutation (`inv_perm[perm[col]]
= col`, computed once) lets `decoded[i] = cp[inv_perm[i]]` be gathered
directly in `O(k)=101`, and the full `O(n)` scatter can be deferred to
the rare candidate that actually passes `verify`. Separately, the
weighted-distance loop (only reached on that same rare pass) was
recomputing `llr[perm[col]] > 0.0` and `.abs()` from scratch on every
call, even though `perm`/`llr` never change across candidates —
precomputed `hdec_perm`/`absrx_perm` once instead. Both changes
bit-identical on host across FT8 (full-parity 8/8, ship-config),
MSK144 and FST4-60, and the full 435-test lib suite.
Re-disassembling: `osd_decode_generic`'s closure shrank from the prior
4.2 KiB down to ~450 bytes, and the scatter/weighted-distance code is
now visibly gated behind the `verify` call rather than running
unconditionally ahead of it.

Measured on real CoreS3 hardware, same 41 baked FST4-60 candidates:

| | before | after | speedup |
|---|---:|---:|---:|
| **`FULL` candidate loop** | **67.789 s** | **59.775 s** | **1.134×** |

2/2 real decodes unchanged (`CQ N5TM EL29`, `CQ K9KFR EN71`).

Cumulative at this point: 89.411 s → 59.775 s, 1.496×, ≈8.5× over the
~7 s budget (down from ≈13×).

**Thirteenth attempt: bit-packing `osd_decode_generic`'s XOR
construction — the one unconditional `O(n)` step the `verify`-gate
reordering above couldn't reach.** That reordering deferred the
scatter to the rare `verify`-passing candidate, but the codeword
*construction* itself (`c1[col] ^= g[k1*n+col]`, and again for
`c2`/`c3`/`c4`) can't be deferred the same way — it's not consuming a
candidate, it *is* the candidate, run unconditionally on every one of
the ~166 650 order-3 combinations. Unlike the scatter, though, no
permutation is involved here: both `c*` and `g`'s rows are already in
the same permuted-column order, so it's a plain elementwise XOR of two
equal-length byte arrays — exactly the shape `osd_setup_generic_packed`
already bit-packs for its own row-XOR during Gaussian elimination
(same file, same reasoning, already in production). Repacking `g`
(byte-per-bit, as `osd_setup_generic_packed` returns it — that
function's own return type stays untouched, since
`packed_setup_differential`'s tests check it against a byte-per-bit
reference) into `OSD_WORDS = 4` `u64` words per row, once per
`osd_decode_generic` call (`O(k·n)`, negligible), turns each `c1`/
`c2`/`c3`/`c4` construction from a 240-byte copy + 240-byte XOR loop
into 4 word-XORs — no copy needed at all, since each is now computed
directly as `parent ^ g_packed[row]`.

`try_and_update` moves to the packed representation too: the `O(k)`
gather (`decoded[i] = cp[inv_perm[i]]`) becomes a bit-extract
(`(cp[idx/64] >> (idx%64)) & 1`) instead of a byte load — same
asymptotic cost, a shift+mask traded for a load — and the scatter +
weighted-distance loop (still `O(n)`, still gated behind `verify`
exactly as before) collapsed from two passes into one, since both now
need the same per-column bit-extract. Bit-identical on host: FT8
(full-parity 8/8, ship-config), MSK144, FST4-60, full 435-test lib
suite, and `fixed-point` feature spot-checked (`ft8_qso3_apoff_recall`
floor unchanged).

Measured on real CoreS3 hardware, same 41 baked FST4-60 candidates:

| | before | after | speedup |
|---|---:|---:|---:|
| **`FULL` candidate loop** | **59.775 s** | **54.087 s** | **1.105×** |

2/2 real decodes unchanged. Stack headroom dropped from ~90 KB to
~83 KB (the new `g_packed` scratch is `[[u64;4]; 256]` ≈ 8 KB) — still
a wide margin against the 96 KB `BENCH_STACK` budget.

**Cumulative from the original 89.411 s baseline: 1.653×, ≈7.7× over
the ~7 s budget (down from ≈13×).** Five small, cheap, disassembly-led
fixes have now closed ~1.65× of the original ~13× gap — real
progress, but still short of a fit, and every individual fix so far
has come in smaller than its own naive justification would suggest
(call-count reduction, element-visit-count unchanged) because the
surrounding per-element work was already there and untouched.
`compute_llr_partial`'s own recursive amplitude-table build
(`build_group_amplitudes`) — the one other identified-but-untouched
hot path, though at ~300 calls/candidate vs. OSD's ~166 650, a much
smaller share — is the last concrete micro-lever left in this class.
Whether the full gap is closable the way WSPR's was (WSPR went
1214.3 s → 249.2 s, 4.9×, across `minsync2` + `opt-level=3` +
`160→240 MHz`) remains open — this measurement answers "does it
currently fit" (no, but less no than before), not "could it with more
of the same kind of work". At ≈7.7×, the gap is close enough to the
size a genuinely different-class lever (fixed-point arithmetic,
dual-core split of the candidate loop, or a recall trade-off) would
plausibly close on its own, where the previous ≈13×/≈9.7× gaps made
that less obviously true.

A mixed-radix or Bluestein implementation for the *larger* FFT sites
this session routed around (issue #307: `coarse_sync`'s 7776,
`downsample_cached`'s 6912, and the equivalent pair for FST4's other
4 submodes) is still open, and secondary to the LLR/BP/OSD cost
question above — closing #307 makes the streaming front end possible,
but a production embedded FST4 path needs the remaining ≈7.7× gap
closed first, or it fits the FFT but still misses the slot.
`DirectDft` itself, being O(N²) and capped at 64, is not meant to grow
into that role.

**Fourteenth attempt: `no8_osd` — the recall trade-off, checked
against real AWGN data before trusting it, then measured.** With the
small disassembly-led fixes exhausted, the recall-trade-off question
came up again: skip `LLR_NSYM_MAX=8` (99% of the LLR/BP-side cost)
outright? The one data point available — this session's own 2 real
decodes never reach `nsym=8` or OSD — is a single-file observation
this project's own discipline says not to trust without checking
against real AWGN data (issue #146/#255's "sparse sampling looks like
a cliff" lesson, among others). `tests/fst4_sweep.rs`'s new
`fst4_60_diag_recall_tradeoff` (real FST4-60 AWGN corpus, 20
trials/SNR) checked it directly, sweeping four configurations —
`full` (today's default), `bp_only` (`DecodeDepth::BP_ONLY`'s shape:
`nsym`-to-8, no OSD), `no8_osd` (`nsym` capped at `LLR_NSYM_MID=4`, OSD
on), `no8_no_osd` (both off) — and found the hypothesis **not
confirmed, in the opposite direction from WSPR's `minsync2`**: near
the -26..-28 dB crossing, both `nsym=8` and OSD carry real,
substantial recall on their own. The more useful (and less intuitive)
finding: `no8_osd` beats `bp_only` at every SNR tested, despite
`nsym=8` being the far more expensive rung to keep — OSD is a
structurally different fallback (bit-flip search over the LDPC
systematic basis) that doesn't need BP to converge at all, so it
rescues candidates every BP LLR variant (including `nsym=8`) misses,
while `bp_only` has no fallback when all of its BP attempts fail.
Per-*trial* (not just aggregate) monotonicity checks — `full ⊇
no8_osd ⊇ no8_no_osd`, `full ⊇ bp_only`, guaranteed by construction —
came back with zero violations across all 200 trials, ruling out a
bookkeeping bug behind the counter-intuitive `no8_osd`-vs-`bp_only`
result.

`no8_osd` wasn't reachable via the existing `DecodeDepth` alone
(`LlrEffort` is FT8-only in practice for the shared pipeline). Added
`skip_llr_nsym_max: bool` to `process_candidate_basic_impl` /
`process_candidate_precomputed` — same additive pattern as `skip_snr`
earlier in this investigation: skips the `nsym=8` BP attempt and its
slot in OSD's variant list, `false` (no behaviour change) for every
existing caller, full merge gate + clippy green.

Measured on real CoreS3 hardware, same 41 baked candidates:

| | `full` | `no8_osd` | speedup |
|---|---:|---:|---:|
| **candidate loop** | **54.087 s** | **25.710 s** | **2.10×** |

2/2 real decodes unchanged. **Cumulative from the original 89.411 s
baseline: 3.48×, ≈3.67× over the ~7 s budget** — by far the largest
single-step win of this whole investigation, and unlike the six
disassembly-led fixes, not "smaller than its own justification
suggested" — but it is a real, conscious sensitivity cost (this is
what `fst4_60_diag_recall_tradeoff` measured), not a free speedup.

Even generous dual-core assumptions don't close the remaining gap on
their own: WSPR's own *measured* dual-core yield was 1.35-1.47× (issue
#260, small-N straggler effects, not the theoretical 2×), and
25.71 s ÷ 1.35–1.47× is still 17.5–19.0 s (≈2.5–2.7× over budget);
even an optimistic 2.0× ceiling only reaches 12.9 s (≈1.84× over).
Untested past this point — recorded as the honest state of the
question, not pursued further this round.

A related side-investigation, prompted by this session's own
disassembly work on `osd_decode_generic`: `osd_decode_deep4`'s
`k4_limit` parameter is documented as restricting order-4's extra
flip to the *least* reliable MRB bits ("errors concentrate in
low-|LLR| bits"), but empirically (a synthetic monotonic-LLR probe
through `osd_setup_generic_packed`) row index `0..k` runs from *most*
reliable (row 0) to *least* reliable (row `k-1`) — meaning the shipped
`0..k4_limit` range is the *most* reliable end, the opposite of the
doc comment's stated intent. Added a `k4_tail: bool` parameter to
`osd_decode_generic` to test both directions directly (`false`
everywhere existing, bit-identical). Traced the production impact
before chasing a fix: `osd_decode_deep4` is FT4-only in practice
(FT8's own bespoke engine uses WSJT-X-faithful `osd_decode_npre1(_
npre2)` instead, never touching this code path — the 3 call sites in
`ft8/decode.rs` are all inside `#[cfg(test)]`), and FT4's own
`osd_depth3_min` gate (`nsync ≥ 14/16`) turned out to structurally
never admit a BP+depth-2/3 failure in clean AWGN — swept m14 through
m22 (20 trials each, near-golden-frequency candidates) and found zero
candidates ever reaching the real depth-4 gate at all, direction
aside. The direction discrepancy is real but currently dormant —
recorded for whenever a future change (a looser gate, CCIR fading, a
new caller) might actually exercise it.

**Fifteenth attempt: WSJT-X-faithful `npre1`/`npre2` OSD, replacing the
unpruned combinatorial search (issue #198).** A collaborator (VK3NV,
issue #306) asked whether `osd_decode_generic`'s plain
k1/k2/k3-combinatorial OSD search — shared by FT4, FST4, and MSK144 —
was even the algorithm WSJT-X itself uses, or whether FT8's own
WSJT-X-faithful `osd_decode_npre1(_npre2)` port (issue #63) should be
the generic one instead. Reading WSJT-X's reference sources directly
confirmed `osd240_101.f90` (FST4) and `osd128_90.f90` (MSK144) use the
*same* `npre1`/`npre2`/`ntheta`/`ntau` pruned-search architecture as
FT8's `osd174_91.f90` — only the tuning constants differ. FST4/MSK144's
`osd_decode_generic` dispatch wasn't just "a different implementation,"
it was a real WSJT-X-fidelity gap (recorded on issue #198).

With `no8_osd` making OSD the dominant remaining cost in the embedded
FST4-60 path, this became a performance question too. Before committing
to the multi-session generic port, a cheap ceiling estimate came first:
`npre1_pattern_counts` (a counting-only port of `osd240_101.f90`'s
`nord=1` pass) measured, on the real FST4-60 golden candidates, `ntotal
= 5151` patterns visited (matching the `k(k+1)/2` closed form for
K=101, and mfsk-core's own FT8 migration's empirical precedent of
`~4,186 = 91×92/2` for K=91) with a mean `npostgate = 11` (0.2%)
actually reaching the expensive full-codeword step — a ~32× reduction
in combinatorial scale versus the unpruned `C(101,3) ≈ 166,650`.

That ceiling justified the port: `osd_decode_npre_generic<P: LdpcParams>`
generalises `osd_npre1_pass`/`osd_npre2_pass` over any `LdpcParams` via
the same `OSD_MAX_N`-bounded fixed-array idiom `osd_decode_generic`
already uses (`ntheta`/`ntau` as runtime parameters — WSJT-X tunes them
per protocol *and* per `ndeep`, so they don't fit as trait constants).
Wired into FST4's OSD dispatch for `ndeep=2` (`ntheta=12`, `npre1`
only) and `ndeep=3` (`+ntau=14`, `npre1+npre2`) — the only two depths
FST4 production code requests.

Not bit-exact with what it replaces (a genuinely different search
strategy), so recall needed its own check rather than a differential
test. An initial AWGN comparison (m26/m27, 100 trials each) against an
*earlier, differently-configured* diagnostic pipeline's baseline
wrongly suggested a 7-point regression at m27 — caught before trusting
it: a new `internal-testing`-only diagnostic override
(`fst4_osd_diag_force_old`) reran the *exact* production entry point
through both algorithms on the same 200 trials for a controlled A/B.
Result: 96/100 vs 96/100 at m26, 71/100 vs 70/100 at m27 — no
measurable recall cost. (The earlier "regression" was an apples-to-
oranges baseline, not a real finding — see this issue's GitHub thread
for the full account.)

Measured on real CoreS3 hardware, same 41 baked candidates, OSD
algorithm as the only variable:

| | old (combinatorial) | new (npre1/npre2) | speedup |
|---|---:|---:|---:|
| **`full`** (nsym=8+OSD) | 54.087 s | **43.134 s** | 1.25× |
| **`no8_osd`** | 25.710 s | **17.147 s** | 1.50× |

2/2 real decodes unchanged in both configurations. **Cumulative from
the original 89.411 s baseline: `full` is now 2.07× (≈6.16× over the
~7 s budget), `no8_osd` is now 5.21× (≈2.45× over budget)** — the
best `no8_osd` figure yet, and confirms VK3NV's own read that the
npre1/npre2 port "looks like the larger potential performance lever."
Dual-core projections improve accordingly: `no8_osd`'s 17.147 s ÷
WSPR's measured 1.35–1.47× dual-core yield is 11.7–12.7 s (still over
budget); the optimistic 2.0× ceiling reaches 8.57 s — close to, but
not yet under, 7 s.

**Caveat, found the same day widening the check to CCIR-moderate
fading:** the "no measurable recall cost" verdict above is AWGN-scoped,
not general. A same-pipeline old-vs-new comparison (100 trials/SNR,
`fst4_osd_diag_force_old`) under CCIR-moderate found a real, growing
decline from m23 through m26 (`full`: 24→18 at m26, -25%; `no8_osd`:
23→18, -22%) that AWGN doesn't show (m26 tied, m27 within 1 trial).
`bp_only`/`no8_no_osd` — configs that never call OSD — are byte-
identical old vs new at every SNR, isolating the cause to the OSD
algorithm itself. Root-caused on a concrete old-only trial: the
unpruned combinatorial search finds a genuine order-3 codeword
(`hard_errors=55/240`, consistent with severe fading-induced
corruption) that `npre1+npre2` misses. Not a port bug — WSJT-X's own
`npre2` hash table only surfaces weight-3 patterns whose partial-
parity signature collides with another MRB pair within the `ntau`-bit
window, not every possible weight-3 combination; under AWGN this
rarely matters (errors concentrate in low-|LLR| bits, matching
`npre1`/`npre2`'s assumption), but a single deeply-faded symbol can
produce an error geometry that assumption doesn't cover. Recorded on
#306/#198; no action taken yet.

**Sixteenth attempt: fresh `FULL`/`BP_ONLY` timing split on the
npre1/npre2 build (VK3NV's request — "where does the bottleneck sit
now?").** `BP_ONLY` doesn't call OSD at all, so it's unaffected by the
port; re-flashed anyway for a same-build number rather than reusing a
pre-port measurement. `full` from the Fifteenth attempt above:

| | time | share of `full` |
|---|---:|---:|
| `BP_ONLY` (LLR/BP only) | 31.023 s | 71.9% |
| `full` − `BP_ONLY` (OSD, inferred) | 12.111 s | 28.1% |
| `full` (both) | 43.134 s | 100% |

Versus the pre-port split (`old full` 54.087 s − same 31.023 s
`BP_ONLY` = 23.064 s OSD, 42.6%): OSD's *absolute* cost dropped 1.90×
(23.064 s → 12.111 s), and its *share* dropped from 42.6% to 28.1% —
the bottleneck has moved back to the BP/LLR side, which the port left
untouched. `BP_ONLY`'s own remaining cost (31.023 s) is still ~4.4×
over the ~7 s budget on its own — the next quantitatively meaningful
lever, if one exists, is back on that side of the split, not OSD.

**Seventeenth attempt: rung-major scheduling + a real `llrd` ablation
(issue #306 item 3, VK3NV).** VK3NV's proposal: schedule by rung
instead of by candidate — try every candidate's `nsym=1` before any
candidate's `nsym=2`, and so on — so a deadline-constrained embedded
receiver reports its easy decodes almost immediately instead of
depending on where they happen to land in the candidate list. Two-step
plan: (1) host-side correctness + a host-timing-projected anytime
curve, (2) a real implementation + real-hardware measurement.

Step 1 found the concept holds: on the real FST4-60 golden recording,
the actual candidate order happens to put both real decodes first
(lucky — depth-first decodes them in ~0.15s/0.30s), but a worst-case
reordering (both decoding candidates moved last) pushes depth-first to
~30.0s/30.2s of a 30.15s total, while rung-major bounds
time-to-first-decode to ~7.4s regardless of order (same total work
either way, confirming the reordering doesn't change *what* gets
decoded, only *when*).

Before committing to a full rung-major implementation, a stage
ablation (`fst4_60_diag_stage_ablation`/`_ccir_moderate`, 100
trials/SNR near crossing) asked whether all six of production's BP/OSD
stages (`llra`, `llrb`, `llre`/mid, `llrc`/max, `llrd`, then OSD on all
five) are worth scheduling at all: **`llrd` (the normalised-nsym=1
variant, tried *last* in production) contributes exactly zero
additional recall**, on both AWGN and CCIR-moderate, in every
configuration tested (byte-identical hit counts with/without it). Not
a rung-major-specific simplification — a genuine 5-stage design for
this protocol, per the user's framing: this pipeline serves FST4
embedded specifically, so it doesn't need to preserve
`engine::pipeline::process_candidate_basic_impl`'s cross-protocol
genericity.

Step 2: `fst4::rung_major::decode_rung_major<P>` — a real,
host-tested, FST4-specific function (not folded into the shared
`process_candidate_basic_impl`, which stays untouched for FT4/FT8/
FST4/MSK144). Five stages, `llrd` dropped, `skip_llrc` mirroring the
existing `no8_osd` trade-off. Verification found a real bug before
trusting any number: the first version never applied the frequency
correction (`freq_shift_cd0` with `df_hz = refined_freq_hz -
cand.freq_hz`) `process_candidate_basic_impl`'s own `try_position`
closure applies — `RungMajorCandidate` had conflated coarse and
refined frequency into one field, silently zeroing `df_hz` and
mis-syncing exactly the marginal candidates this whole exercise is
about. Surfaced because a first real-hardware total (12.500 s) looked
implausibly good against the known 43.134 s baseline; a direct
nsync-gate candidate count against the actual baked embedded asset
found only 4 of 41 candidates clearing the gate instead of the
expected 6, confirming the bug rather than a genuine algorithmic
win — fixed, and the 6 expected hard candidates reappeared with
sensible per-stage timings.

**Corrected real-hardware measurement** (same 41 baked candidates,
`llrd` dropped, `skip_llrc` as the only other variable):

| | 6-stage (production shape) | 5-stage (`decode_rung_major`) | speedup |
|---|---:|---:|---:|
| `full` | 43.134 s | **40.102 s** | 1.08× |
| `no8_osd` | 17.147 s | **13.643 s** | 1.26× |

2/2 real decodes unchanged. **Cumulative from the 89.411 s baseline:
`no8_osd` is now 6.55×, ≈1.95× over the ~7 s budget** — the best
figure this investigation has produced. Dual-core projections: 13.643 s
÷ WSPR's measured 1.35–1.47× yield is 9.28–10.1 s (still over budget);
the optimistic 2.0× ceiling reaches **6.82 s — under the 7 s target
for the first time in this investigation**, though only at that
ceiling, not the measured dual-core yield.

Rung-major scheduling itself (the ordering property, as opposed to the
`llrd` ablation) hasn't yet been measured on real hardware in isolation
— `decode_rung_major` runs the stages in rung-major order by
construction, but this session's real-hardware numbers above measure
its *total*, not a depth-first-vs-rung-major latency comparison on
device the way the host projection did. That would need a depth-first
counterpart built on the same 5-stage set for a fair on-device A/B —
not done this round.

**Eighteenth attempt: issue #308's `i0` jitter retry — made a caller
choice instead of a fixed policy.** The same day #308 landed on host
(FST4 candidates now retried at `i0 ∈ {refined, +1, -1}`, matching
WSJT-X's `fst4_decode.f90`), it was ported into `decode_rung_major` too
— straightforward given both go through the same `symbol_spectra`/BP/
OSD building blocks. Real-hardware measurement found the cost
disproportionate to the recall gain if applied unconditionally: `full`
40.102 s → **121.281 s** (3.02×), `no8_osd` 13.643 s → **34.200 s**
(2.51×), versus a recall gain of only a few points at the SNRs checked
(`fst4_60_diag_i0_offset_ablation`: AWGN m27 74→82/100, CCIR-moderate
m26 18→23/100 — real, not proportional to a 2-3× cost). An ablation of
the two offsets individually found neither is a clean free cut the way
`llrd` was: `{0,-1}` alone captures most of the combined benefit
(AWGN m27 79/100, CCIR-moderate m26 21/100) but still costs a real
1.84× on a quick host-timing read (`fst4_60_diag_i0_offset_host_
timing`), and `{0,+1}` captures less while costing about the same.

Rather than mfsk-core picking one answer, `decode_rung_major` exposes
`offsets: &[i32]` as an explicit caller parameter — same shape as the
existing `skip_llrc`/`skip_osd` choices. A monitoring-style deployment
that can tolerate spanning slots has a very different cost/recall
trade-off than one with a hard per-slot deadline, so this is a
deployment decision, not a decoder one. The 2-arg `decode_rung_major`
wrapper keeps `&[0]` as the deadline-tight default (`no8_osd`: 13.643 s,
≈1.95× over the ~7 s budget, the same figure the Seventeenth attempt
closed with); `&[0, -1]` and `&[0, 1, -1]` are available to a caller
that wants more recall and can spend more time, with the cost/recall
numbers above already measured rather than needing to be re-derived.
Host (`process_candidate_basic_impl`, issue #308, unconditional `i0±1`)
and embedded's *default* are still allowed to diverge — WSJT-X-fidelity
and a hard embedded deadline are different questions — but embedded is
no longer locked out of the fidelity fix if a given deployment can
afford it.

## FT4 on embedded

**Status (2026-08-30, superseded — see [Where this stands
(2026-09-01)](#where-this-stands-2026-09-01--a-receiver-inside-its-budget)
at the end of this section): builds, decodes correctly on hardware, and is
3.4× over its slot budget** after the 2026-08-29 optimisations (8.8×
before them). The remaining excess is spread across three stages, one
of which — `downsample_cached` — a host-verified DDC front end now
removes outright.

### What it took to build at all

`ft4 = []` in `mfsk-core/Cargo.toml` has always claimed FT4 is
backend-agnostic, and it is — `src/ft4/` is 745 lines of trait impls
and config over `engine::pipeline`, with no `rustfft` and no
FT8-specific reference anywhere. But no embedded crate had ever
enabled the feature, and neither `scripts/pre-push-check.sh` nor
`ci.yml` carried an `alloc ft4 fft-extern` rung, so the claim had
never been tested. The first `cargo check` against it failed on
exactly one line — `ft4/subtract.rs`'s `Vec` with no
`use alloc::vec::Vec;`, byte-for-byte the gap issue #306 found twice
in FST4. Both matrices now carry the rung.

Two FFT lengths stood between FT4 and a board, both non-power-of-two:

| length | where | resolution |
|---|---|---|
| `fft1_size = 92_160` | `build_fft_cache`, once per slot | **baked on host**, fed through `decode_frame`'s `precomputed_fft` seam — the same escape FST4 uses |
| `fft2_size = 5_120` | `downsample_cached`, once per candidate | `engine::dsp::fft_mixed_5120` — Cooley-Tukey 1024 × 5, reusing the existing `fft_15::fft_5` kernel, the same shape as `fft_mixed_3840`'s 256 × 15 |

`engine::llr::symbol_spectra`'s per-symbol DFT needs no new kernel:
FT4's `ds_spb = NSPS/NDOWN = 32` is a power of two. (`ft4_coarse_sync`'s
own `NFFT1 = 2304` = 256 × 9 needed one more wrapper of the same shape;
`engine::dsp::fft_mixed_2304` is it, added 2026-08-30 — the bench still
bakes the candidate list, and that stage is 0.3 ms of the host slot.)

### The budget

FT4's slot is 7.5 s. Transmission starts at 0.5 s and runs 105 symbols
× 48 ms = 5.04 s, so the frame ends at 5.54 s and **1.96 s** is left to
decode in. Unlike WSPR's and FST4's monitor loops — built with
deliberate slack, where an overrun is a fault — this is the same shape
of budget FT8's 15 s slot has: genuinely tight, and an overrun is an
operating limit.

### Measured

`ft4-bench`, M5Stack CoreS3 @ 240 MHz, `opt-level = 3`, single core, no
WiFi. 31 coarse candidates from the WSJT-X golden `000000_000002.wav`,
single pass. Log: `embedded-poc/m5stack-cores3-app/logs/
ft4-bench_clean_2026-08-29.log`.

| stage | total | per candidate | share |
|---|---:|---:|---:|
| `downsample_cached` (5120-pt inverse FFT) | 2 252 ms | 72.7 ms | 13 % |
| **`ft4_sync_search`** | **13 225 ms** | **424 ms** | **76 %** |
| LLR + BP (`DecodeDepth::EMBEDDED`) | ~1 861 ms | ~60 ms | 11 % |
| **total, production call** | **17 339 ms** | 559 ms | — |
| same at `DecodeDepth::FULL` | 19 684 ms | 635 ms | — |

**11 distinct decodes, identical to the host on the same assets**, at
both depths — so the ship config gives up no recall here, and OSD buys
nothing on this file. Memory was never in question: 7.47 MB PSRAM and
240 KB internal DRAM free throughout, and the bench task used 16.7 KB
of its 96 KB stack.

**17 339 ms against 1 960 ms is 8.8× over.**

### The bottleneck is structural, not statistical

`ft4_sync_search`'s per-candidate cost across all 31: **min 423 835 µs,
p50 423 897 µs, max 424 684 µs** — a 0.2 % spread. That is the
signature of a fixed grid, not of anything candidate-dependent:
`ft4_sync_search_window` walks the same absolute `[-344, 1012]`
downsampled-sample window for every candidate regardless of its own
`dt_sec` (a faithful port — WSJT-X's FT4 decoder determines Δt here and
nowhere else), scoring ~19 900 (Δf, Δt) cells of 4 Costas blocks × 4
symbols × 32 samples each. That is ~10.2 M complex MACs per candidate,
and 424 ms of it works out to roughly 10 cycles per complex MAC — a
scalar f32 inner loop with an on-the-fly phasor rotation.

So the levers are the grid and the arithmetic inside it, and both are
measurable before either is attempted:

- **`dsps_dotprod_f32_aes3`** (LX7 PIE) on the inner product. The
  `dotprod-bench` in this crate already measured what PIE is worth on
  this chip; ~10 cycles/MAC is a long way from what the kernel can do.
- **Narrowing the window** — **measured on host, 2026-08-29**, see
  `docs/notes/FT4_BENCHMARK.md` §18. The production window is ±1.0 s,
  not a full slot (`i0` is downsampled samples; `dt = 0` sits at
  `i0 = 333`). Two instruments agree on **±0.5 s being free**: on the
  real off-air golden all 11 decodes survive (their true DTs span
  −0.44 … +0.30 s) at a measured **1.91×** on the search, and an
  `ft4sim` DT sweep shows a hard cliff exactly at the window edge —
  100 % inside, 0 % outside, and *identical* recall column-to-column
  near threshold wherever the DT is inside. Narrowing costs **reach,
  not sensitivity**.

### Both levers applied, and a third that measured smaller than it looked

All three are in, measured on the same 31 candidates
(`logs/ft4-bench_opt_2026-08-29.log`, full account in
`docs/notes/FT4_BENCHMARK.md` §19):

| configuration | search | cumulative |
|---|---:|---:|
| baseline | 13 225 ms | 1.00× |
| + `FlatRef` / `dot_f32` | 4 447 ms | **2.97×** |
| + `cd0` in internal DRAM | 3 937 ms | 3.36× |
| + ±0.5 s window | **2 492 ms** | **5.31×** |

**The arithmetic was the win.** `ft4_sync_search_window` applied its
frequency shift inside the innermost sample loop, restarting a rotating
phasor at every `(df, i0)` cell — but that phasor is indexed by offset
*within the Costas block*, so it was identical across all ~340 `i0`
positions per `df`. `fst4_sync_search` was already folding it into the
reference (`FlatRef`), which also leaves a plain inner product that
`dot_f32` — hence `dsps_dotprod_f32_aes3` — can serve. FT4 now does the
same. Sensitivity unmoved (all four sweep channels +0.00 dB) and the
golden's stage counters identical.

**The PSRAM hypothesis was wrong: 1.12×, not the 5–10× predicted.** The
byte count was right and the inference was not — after the change the
access is a sequential `dot_f32` over 2 KB slices, which the S3's PSRAM
cache serves well; the old loop was compute-bound, not bandwidth-bound.
Kept (40 KB, free once reserved at boot) but it is not a lever. A
production FT4 mode would need it through `worker_arena` at boot
regardless: with WiFi up the largest free internal block here is
31 744 B.

**Slot total, production path: 17 339 ms → 8 642 ms (2.01×)**, still 11
decodes matching the host at both depths. With all three applied the
projection is ~6 686 ms against 1 960 ms — **3.4× over, from 8.8×**.

What is left is no longer one thing: downsample 34 % / search 37 % /
LLR+BP 29 %. `downsample_cached` has become co-equal with the search,
and that is a stage a DDC front end removes rather than speeds up —
which is what the next section is.

### The DDC front end (host-verified 2026-08-30, not yet on hardware)

`mfsk_core::ft4::ddc` builds the per-candidate `cd0` by mixing and
filtering, so the 92 160-point transform this bench bakes on a host has
nothing left to feed. Full account in `docs/notes/FT4_BENCHMARK.md` §20.

**FT4 is the easy case.** `fst4::ddc` needs a rational resampler
because `NSPS = 3888 = 2⁴·3⁵` leaves a `3⁵` denominator; FT4's
`NDOWN = 18` divides 12 kHz exactly, `666.667 Hz` is already
`SyncDims::ds_rate`, and `ds_spb = 32` is a power of two. The module is
two `FirStage`s and two mixers — no `PolyphaseResampler`, no `RxGrid`,
nothing downstream of `cd0` changed:

```text
12 kHz real i16
  → Mixer(f0 + 31.25 Hz)                   complex @ 12 kHz
  → FirStage A: 199 taps, fc 320 Hz, ÷18   complex @ 666.667 Hz
  → FirStage B: 263 taps, fc 56 Hz,  ÷1    complex @ 666.667 Hz
  → Mixer(−31.25 Hz)                       cd0, f0 at DC
```

**The passband is a decode parameter, not a filter-design free choice.**
`downsample_cached` keeps `[f0 − 31.25, f0 + 93.75] Hz` and zeroes the
rest — asymmetric about `f0`, because the tones run upward from it.
`process_candidate_basic_impl` then RMS-normalises `cd0` over its whole
length, and `LLR_SCALE` is calibrated against that, so noise admitted
outside the reference band rescales every LLR feeding BP (the full
±333 Hz baseband would have been ~2.3× high). Hence the mixer pair:
centre the *band*, filter symmetrically with real taps, rotate `f0`
back to DC. Measured equivalent noise bandwidth against the reference:
**+0.021 dB**.

**Equivalence.** On the WSJT-X golden, from the same 31 candidates:
11 distinct decodes on both front ends, identical sets, at both
`DecodeDepth::EMBEDDED` and `FULL`; the refined sync position never
moves, and one candidate of eleven lands one `ft4_sync_search` grid
step (1 Hz) away. On the tier-C sweep, paired on the same 560 noise
realisations across four channels' 50% crossings: **FFT 237 decodes,
DDC 238**, five disagreements split three/two. The swap costs 0.0 dB.

### The candidate budget was 2.6x too big (2026-08-30)

Every stage after `ft4_coarse_sync` is per-candidate, and the bench's
search had been passing `sync_min = 0.05`. That is *below the noise
floor*: `getcandidates4.f90` divides the smoothed spectrum by a fitted
baseline, so noise sits at ~1.0 and any lower threshold admits every
peak in the band. WSJT-X's own value is 1.2 (`ft4_decode.f90:195`).

Measured over 560 sweep files straddling four channels' 50% crossings
plus the golden recording: 0.05 → 1.2 takes the candidate count from
67.1 to 1.6 (sweep) and 31 to 12 (golden) **with identical recall on
both**; the knee is at 1.4, where the first decodes start dropping.
`bench_assets::SYNC_MIN` is now 1.2 and the baked candidate list was
re-generated — 31 → 12, same 11 decodes at both depths. Sections 17-19
of `FT4_BENCHMARK.md` were all measured over 31 candidates.

Combined with the DDC, the projection is
`(2 492 + 1 943) × 12/31 ≈ 1 717 ms` against a 1 960 ms budget — inside
it for the first time, **on paper**, with neither change measured on the
board.

The same measurement corrected a claim in the other direction: the
bench's "`EMBEDDED` and `FULL` decode identically" is true on the golden
and false on weak data (237 vs 179 of 560 at the crossing), so the ship
depth does give up about a quarter of its recall to skip OSD. See
`docs/notes/FT4_BENCHMARK.md` §21.

Like `fst4::ddc`, this is a building block callers reach for, not a
feature flag that swaps the host's front end.

### Where this stands (2026-09-01) — a receiver, inside its budget

**The 3.4× at the top of this section is superseded**, and so is the
"no hardware measurement" that used to close it. Everything projected
above has since been built and run on a CoreS3; the account is
`docs/notes/FT4_BENCHMARK.md` §32-§34, §37-§38 and §42, and the
summary is:

| change | what it did |
|---|---|
| `Ft4SavgBuilder` — the coarse stage runs *during* capture (§32) | 761 ms → 6 ms after the slot closes |
| a candidate loop held to a slot deadline (§34) | the overrun became an operating choice instead of a fact |
| the shared decimation (§42) | ~188 → ~168 ms per candidate |
| streaming that decimation from the capture path (§42.1) | slot 2 067-2 118 ms → **1 998-2 000** |
| re-deriving the budget from key-up, not the slot end (§43) | the QSO-capable budget was **500 ms**, not 1 960 |
| two cores, candidates from a shared cursor (§44) | 1.40× on the candidate loop |
| task stacks sized from measurement (§45) | **1 290-1 401 ms with WiFi associated** |

**The budget is key-up, not the slot boundary.** FT4 is a fast-QSO
mode, so the deadline is the moment the station must transmit — 0.5 s
into the next slot — and the capture window closes as soon as the
audio the search can reach has arrived (6.25 s of 7.5 s, including the
DDC chain's group delay). Anchored to the slot end instead, a
transmitting build had 500 ms to decode in. See §43 for the timeline.

**Stacks are internal DRAM, and internal DRAM is what WiFi takes.**
The decode tasks asked for 32 KB each and used 2.6-4.5 KB; with WiFi
associated that waste pushed the largest free internal block to
31 744 B and the decoder's own allocations into PSRAM, costing
400-580 ms a slot. Stopping the radio does *not* fix it
(`esp_wifi_stop` frees nothing); sizing the stacks does. §45.

`ft4-demo` on the replayed golden slot — the 14-signal, FT8-density
pessimum — now runs **12 of 12 candidates and decodes 11**, against 11
of 12 and 10 before the shared front end, at the 1 960 ms transceiver
budget. At the 5-10 signal occupancy FT4 actually sees (§23) the loop
finishes with nothing cut.

**The shared decimation is the front end's second half.**
`ft4::ddc`'s per-candidate chain used to filter all 90 000 samples of
the slot at 12 kHz for every candidate. `NDOWN = 18` factors as
`2 · 9`, so `SlotDecimator` (165 taps, ÷2, real input through
`FirStage::push_block_real`) runs once per slot and
`CandidateDdc::new_half_rate` runs the same chain in Hz at 6 kHz with
101 taps. The corner is 2 800 Hz rather than 3 000 because content
above the new Nyquist folds *into* a candidate's band, and 165 taps
rather than 111 because a 2 700 → 3 300 transition does not protect the
top of the search band. Equivalent noise bandwidth against the
reference is +0.021 dB with the extra stage and +0.021 dB without.

**What is still missing**: no esp-dsp binding for
`FirStage::push_block` (`dsps_fird_f32_aes3`); the 560-file paired
sweep for the shared front end is written but not yet run; the FT4
boot mode has not been run against a radio (it replays a baked golden
slot by default, `MFSK_FT4_REPLAY=0` turns that off); and the
real-audio path still has no wall-clock slot alignment, the open item
`uac.rs` shares with #313.

## Live UAC bring-up — what to check, in what order (issue #163)

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

Needs the open item this procedure does *not* cover: **the real-audio
path has no wall-clock slot alignment**. Slot boundaries are counted in
samples from whenever the stream started, so a live band decode
requires the transmission to happen to line up, or the NTP-fed
alignment hook (`uac.rs`'s own open item, tracked in #313) to land
first. Checkpoint 1 does not care — the golden recording is a whole
slot and the decoder finds its own `dt` — which is exactly why it is
worth doing first.

## Where to go next

By reader intent:

- **I want to operate an existing FT8 controller** →
  [`docs/reference/MANUAL_M5STICKS3.md`](MANUAL_M5STICKS3.md) (build / flash /
  `cfg.toml` / `BootMode` cycle / UI / QSO workflow / troubleshooting).
- **I want to integrate `mfsk-core` on a new MCU** → start with
  the [FFT extern contract](#the-fft-extern-rust-contract), then
  the [`mfsk-ffi-ft8` C ABI](#using-from-c--c--non-rust-esp-idf-projects-mfsk-ffi-ft8)
  if you're calling from C. The
  [`embedded-poc/embedded-shared/src/esp_dsp_fft.rs`](https://github.com/jl1nie/mfsk-core/blob/main/embedded-poc/embedded-shared/src/esp_dsp_fft.rs)
  shim is the worked example to copy from.
- **I want to contribute to one of the embedded apps** →
  [`embedded-poc/CLAUDE.md`](https://github.com/jl1nie/mfsk-core/blob/main/embedded-poc/CLAUDE.md)
  for the cross-board toolchain notes + LX6/LX7 comparison table,
  then the per-crate `CLAUDE.md` for board-specific gotchas.
- **I want to track the embedded roadmap** →
  [`docs/notes/ROADMAP.md`](../notes/ROADMAP.md) Phase B-Stick (M5StickS3 demo /
  acoustic fallback) and Phase B-Core (M5Stack CoreS3 main UAC
  controller) sections.
- **I want WSPR, not FT8** → [WSPR on embedded](#wspr-on-embedded)
  above, then `docs/notes/ROADMAP.md` Phase E and
  [`docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`](../notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md)
  for the full measurement journal.
