# Embedded targets

`mfsk-core` is `no_std + alloc` capable: the FT8 decode path
(`mfsk_core::ft8::decode_block`) runs on chips with as little as
~150 KB of usable RAM when paired with a caller-supplied FFT
backend. This document is the reference for embedded integrators —
what the library asks of the caller, what scratch buffers are
needed, how the C ABI is shaped, and what performance to expect on
the targets we exercise today.

For host-only usage (no embedded) see [`docs/reference/LIBRARY.md`](LIBRARY.md);
for operating an existing FT8 controller built on this library see
[`docs/reference/MANUAL_M5STICKS3.md`](MANUAL_M5STICKS3.md).

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
| LDPC BP NMS (`fec::ldpc::bp`) | `LlrScalar` | ✅ via `fixed-point` |
| LLR computation (`engine::llr`) | `SpecScalar` × `LlrScalar` | ✅ via `fixed-point` |
| BP scratch pool (`BpScratch<P, T>`) | `LdpcParams` × `LlrScalar` | ✅ — works for FT8 LDPC(174,91) and FST4/uvpacket LDPC(240,101) |
| FT8 spectrogram + DFT (`ft8::decode_block`) | `SpecScalar` × `AudioSample` | ✅ via `fixed-point` |
| WSPR (`wspr::decode`, `wspr::ddc`) | — | ❌ — runs plain host f32 on embedded too, via `fft-extern`; never needed the integer path. See [WSPR on embedded](#wspr-on-embedded) below. |
| **FT4 / Q65 / JT9 / JT65** | (host f32 only) | ❌ — these protocols don't go through `decode_block` today, and have no embedded path at all yet |

So: **the trait infrastructure is protocol-agnostic, but the only
protocol that actually flips into the integer path on the embedded
build is FT8.** Adding FT4 (next-most-likely candidate, since it
shares the same Costas/Gray/LDPC pieces) is a port of the
`decode_block` shape to FT4-specific symbol layout — nothing new
in the trait layer.

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
| **M5Stack CoreS3** | ESP32-S3 LX7 + AXP2101 PMIC + AW9523B I/O expander (BUS_OUT_EN on P1 drives VBUS boost) | esp-dsp `_ae32_` asm (same Phase D D1 migration applies) | **Main UAC controller target** (Phase B-Core, 2026-05-17 pivot) — `embedded-poc/m5stack-cores3-app/`. Phase 0-Core (bringup) + Phase 1-Core (AW9523B BUS_OUT_EN + UAC host) shipped in commit `1a93c92`. M5StickS3 cannot do USB-OTG host (no VBUS source circuit), so it was repositioned as the **demo / acoustic-fallback** board and the live USB Audio Class path to IC-705 lands on CoreS3 instead. See `docs/notes/ROADMAP.md` Phase B-Core. |

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
    "fixed-point",      # u16 spec + i16 DFT + Q11i16 LLR + integer NMS BP
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
| `fixed-point` | Embedded integer pipeline: u16 spectrogram + i16 internal DFT + Q11i16 LLR + integer NMS BP. Implies `nstep-half`. (Was `Q3i8` in 0.5.x — 0.6.2 widened the LLR to `Q11i16` because host fixed-point + rustfft hit 16/18 on `qso3_busy.wav` with f32 but only 9/18 with `Q3i8`; the resolution step was the recall ceiling, not anything DSP-side. `Q3i8` stays in `engine::scalar` for the comparison path.) | Any embedded target — close to host f32 recall (1/2048 LSB LLR resolution), halved PSRAM bandwidth, ~12 KB BP scratch (Q11i16, post-0.6.2). |
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
| LLR | f32 (host) or **Q11i16** (`fixed-point`, since 0.6.2 — was `Q3i8` in 0.5.x; widened to address the resolution-limited recall ceiling) | f32 unbounded; Q11i16 ±16 with ~1/2048 LSB (Q3i8 ±16 with ~1/8 LSB stays in `engine::scalar` for the comparison path) | `engine::scalar::LlrScalar` |
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

**Not yet verified**: live audio capture. Everything above is
measured against a WAV-fed / synthetic baseband, the same gap issue
[#163](https://github.com/jl1nie/mfsk-core/issues/163) tracks for the
FT8 controller line — UAC hardware verification is a shared, still-open
dependency for both. `mfsk_app_shared::wsprnet` (wsprnet.org spot
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
   re-flashing `wspr-app` after this change resets its
   littlefs-stored settings.

With both fixed, the app boots and loads its baked assets cleanly
(1 008 ms for 1.44 MiB audio + 5.7 MiB `fft_cache`, PSRAM headroom
fine — 893 KiB free post-load against the ~14.3 MiB combined
Vec+baked-bytes peak the module doc flagged as the open risk; that
risk did not materialize). It then panics on the very first FFT call,
inside `coarse_sync`'s spectrogram stage, before either the PSRAM
question or a wall-clock number could be reached:

```
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

**So the honest answer to issue #306: no, not as measured today** —
89.7 s against a ~7 s budget is not a rounding error, and closing a
13× gap needs real optimization work, not a bigger margin somewhere
else. Whether that gap is closable the way WSPR's was (WSPR went
1214.3 s → 249.2 s, 4.9×, across `minsync2` + `opt-level=3` +
`160→240 MHz`, none of which had an FST4 equivalent applied here
beyond the two already baked into this build) is an open question this
measurement doesn't answer by itself — it answers "does it currently
fit" (no), not "could it".

A mixed-radix or Bluestein implementation for the *larger* FFT sites
this session routed around (issue #307: `coarse_sync`'s 7776,
`downsample_cached`'s 6912, and the equivalent pair for FST4's other
4 submodes) is still open, and now secondary to the LLR/BP/OSD cost
question above — closing #307 makes the streaming front end possible,
but a production embedded FST4 path needs the ≈13× gap closed first,
or it fits the FFT but still misses the slot. `DirectDft` itself,
being O(N²) and capped at 64, is not meant to grow into that role.

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
