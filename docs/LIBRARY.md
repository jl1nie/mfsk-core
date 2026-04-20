# mfsk-core — Library Architecture & API Reference

> **日本語版:** [LIBRARY.ja.md](LIBRARY.ja.md)

This document covers the mfsk-core library surface for embedders:
Rust crate consumers, C/C++ projects linking `libmfsk.so`, and
Kotlin/Android apps using the JNI scaffold.

For a quick-start overview (badges, dependency snippet, minimal
example) see [README.md](../README.md). This document goes deeper
into *why* and *how*.

## 0. Introduction

### 0.1 Background

The weak-signal digital modes addressed by this library — FT8, FT4,
FST4, WSPR, JT9, JT65 and their siblings — were developed by Joe
Taylor K1JT and his collaborators as part of the WSJT-X project,
which is the reference implementation for the entire family. Every
algorithm in mfsk-core (sync correlation, LLR computation, LDPC
BP / OSD decoding, Fano sequential decoding of convolutional codes,
Reed-Solomon erasure decoding, per-protocol message encoding, …) is
derived from WSJT-X. Each source file's docstring cites the
corresponding file under `lib/ft8/`, `lib/ft4/`, `lib/fst4/`,
`lib/wsprd/`, `lib/jt9_*`, `lib/jt65_*`, etc.

WSJT-X evolved as a C++ + Fortran desktop application, and has been
refined in that form over many years. Deploying those same
algorithms outside the desktop — running them in a browser PWA,
embedding them in a standalone Android app, or calling them as a
library from another Rust or C++ project — requires a non-trivial
amount of per-platform work if one starts from the upstream source.

### 0.2 Goal

mfsk-core re-implements the WSJT-X algorithms in Rust and organises
them as a single crate that can be consumed identically from several
runtimes (native Rust, WebAssembly, Android JNI, C ABI). The aim is
to keep algorithmic equivalence with the upstream C++/Fortran code
while broadening the set of platforms that can host it.

### 0.3 Design approach

Protocol-independent algorithms — DSP, sync, LLR, the equaliser,
LDPC BP / OSD, Fano convolutional decoding, Reed-Solomon erasure
decoding, and the shared parts of the message codec — live in the
`core`, `fec`, and `msg` modules. Each protocol is a comparatively
small zero-sized type (ZST) that declares its own constants and the
specific FEC / message codec it uses. The pipeline is expressed as
`decode_frame::<P>()`, taking `P: Protocol` as a compile-time type
parameter so that monomorphisation produces specialised code per
protocol. The abstraction does not add runtime cost.

Some direct consequences of this approach:

- The same algorithm implementation runs under native Rust, WASM,
  Android, and C / C++.
- Improvements to a shared path (e.g. LDPC BP) automatically benefit
  every protocol that uses it.
- Adding a new protocol tends to keep the diff confined to that
  protocol's own code (see §2 for the concrete steps).
- The C ABI in `mfsk-ffi` branches only once via `match protocol_id`;
  past that point, the code is already specialised.

### 0.4 Currently supported protocols

| Protocol   | Slot    | FEC                          | Message | Sync                 | Upstream source |
|------------|---------|------------------------------|---------|----------------------|-----------------|
| FT8        | 15 s    | LDPC(174, 91) + CRC-14        | 77 bit | 3×Costas-7           | `lib/ft8/`      |
| FT4        | 7.5 s   | LDPC(174, 91) + CRC-14        | 77 bit | 4×Costas-4           | `lib/ft4/`      |
| FST4-60A   | 60 s    | LDPC(240, 101) + CRC-24       | 77 bit | 5×Costas-8           | `lib/fst4/`     |
| WSPR       | 120 s   | convolutional r=½ K=32 + Fano | 50 bit | per-symbol LSB       | `lib/wsprd/`    |
| JT9        | 60 s    | convolutional r=½ K=32 + Fano | 72 bit | 16 distributed slots | `lib/jt9_decode.f90`, `lib/conv232.f90` |
| JT65       | 60 s    | Reed-Solomon(63, 12) GF(2⁶)   | 72 bit | 63 distributed slots (pseudo-random) | `lib/jt65_decode.f90`, `lib/wrapkarn.c` |

### 0.5 Checking that the design actually works — using WSPR

FT8, FT4 and FST4 share so much (LDPC FEC, 77-bit messages, block
Costas sync) that their common code is unavoidable rather than a
test of the abstraction. WSPR, in contrast, differs from the FT
family in three structural ways, which makes it a useful check on
whether the abstraction holds up.

1. **Different FEC family** — convolutional (r=1/2, K=32) with Fano
   sequential decoding instead of LDPC. Added as
   `mfsk_core::fec::conv::ConvFano`.
2. **Different message length** — 50 bits instead of 77. Types 1, 2
   and 3 are implemented in `mfsk_core::msg::wspr::Wspr50Message`.
3. **Different sync structure** — the lower bit of every channel
   symbol carries one bit of a fixed 162-bit sync vector, so sync is
   not a block of Costas arrays. Captured by adding an `Interleaved`
   variant to `FrameLayout::SYNC_MODE`.

Each of these touched a different axis of the trait surface. All
three were absorbed by adding new implementations / variants, and
the FT8 / FT4 / FST4 code paths were left alone. In practice, those
three protocols still use `SyncMode::Block` and emit the same bytes
they did before.

## 1. Module layout

```
mfsk_core
├── core/             Protocol traits, DSP, sync, LLR, equaliser, pipeline
│   ├── protocol.rs     ModulationParams / FrameLayout / Protocol / FecCodec / MessageCodec
│   ├── dsp/            resample · downsample · gfsk · subtract
│   ├── sync.rs         coarse_sync / refine_candidate
│   ├── llr.rs          symbol_spectra / compute_llr / sync_quality
│   ├── equalize.rs     equalize_local (Wiener per-tone)
│   └── pipeline.rs     decode_frame / decode_frame_subtract / process_candidate_basic
├── fec/              FecCodec implementations
│   ├── ldpc174_91.rs   LDPC(174, 91)  — FT8, FT4
│   ├── ldpc240_101.rs  LDPC(240, 101) — FST4
│   ├── conv.rs         ConvFano r=½ K=32 — WSPR, JT9
│   └── rs63_12.rs      RS(63, 12) GF(2⁶) — JT65
├── msg/              Message codecs
│   ├── wsjt77.rs       77-bit WSJT message (pack / unpack) — FT8, FT4, FST4
│   ├── wspr.rs         50-bit WSPR Types 1 / 2 / 3
│   ├── jt72.rs         72-bit JT message — JT9, JT65
│   └── hash.rs         Callsign hash table
├── ft8/              FT8 ZST + decode + wave_gen
├── ft4/              FT4 ZST + decode
├── fst4/             FST4-60A ZST + decode
├── wspr/             WSPR ZST + decode + synth + spectrogram search
├── jt9/              JT9 ZST + decode
└── jt65/             JT65 ZST + decode (+ erasure-aware RS)
```

Each protocol module is gated behind a feature flag (`ft8`, `ft4`,
`fst4`, `wspr`, `jt9`, `jt65`). The `core`, `fec`, and `msg`
modules are always available.

The `mfsk-ffi` sibling crate in this workspace builds a C ABI
shared library (`libmfsk.{so,a,dylib}` + `mfsk.h`) on top of the
same crate.

## 2. Protocol trait hierarchy

Every supported mode is described by a zero-sized type that
implements three composable traits:

```rust
pub trait ModulationParams: Copy + Default + 'static {
    const NTONES: u32;
    const BITS_PER_SYMBOL: u32;
    const NSPS: u32;              // samples/symbol @ 12 kHz
    const SYMBOL_DT: f32;
    const TONE_SPACING_HZ: f32;
    const GRAY_MAP: &'static [u8];
    const GFSK_BT: f32;
    const GFSK_HMOD: f32;
    const NFFT_PER_SYMBOL_FACTOR: u32;
    const NSTEP_PER_SYMBOL: u32;
    const NDOWN: u32;
    const LLR_SCALE: f32 = 2.83;
}

pub trait FrameLayout: Copy + Default + 'static {
    const N_DATA: u32;
    const N_SYNC: u32;
    const N_SYMBOLS: u32;
    const N_RAMP: u32;
    const SYNC_MODE: SyncMode;  // Block(&[SyncBlock]) or Interleaved { .. }
    const T_SLOT_S: f32;
    const TX_START_OFFSET_S: f32;
}

pub enum SyncMode {
    /// Block-based Costas / pilot arrays at fixed symbol positions.
    /// Used by FT8 / FT4 / FST4.
    Block(&'static [SyncBlock]),
    /// Per-symbol bit-interleaved sync: one bit of a known sync vector
    /// is embedded at `sync_bit_pos` within every channel-symbol tone
    /// index. Used by WSPR (symbol = 2·data + sync_bit).
    Interleaved {
        sync_bit_pos: u8,
        vector: &'static [u8],
    },
}

pub trait Protocol: ModulationParams + FrameLayout + 'static {
    type Fec: FecCodec;
    type Msg: MessageCodec;
    const ID: ProtocolId;
}
```

### Worked examples — how the traits compose

Two concrete cases show how the three traits combine on a real ZST.

**FT4** — a standard block-Costas protocol that shares its FEC and
message codec with FT8:

```rust
use mfsk_core::core::protocol::*;
use mfsk_core::fec::ldpc174_91::Ldpc174_91;
use mfsk_core::msg::wsjt77::Wsjt77Message;

pub struct Ft4;

impl ModulationParams for Ft4 {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 576;          // 48 ms @ 12 kHz
    const TONE_SPACING_HZ: f32 = 20.833;
    const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2];
    // … (GFSK_BT, NFFT_PER_SYMBOL_FACTOR, NDOWN, …)
}

impl FrameLayout for Ft4 {
    const N_DATA: u32 = 87;
    const N_SYNC: u32 = 16;
    const N_SYMBOLS: u32 = 103;
    const SYNC_MODE: SyncMode = SyncMode::Block(&FT4_SYNC_BLOCKS);
    const T_SLOT_S: f32 = 7.5;
    const TX_START_OFFSET_S: f32 = 0.5;
}

impl Protocol for Ft4 {
    type Fec = Ldpc174_91;          // shared with FT8
    type Msg = Wsjt77Message;       // shared with FT8
    const ID: ProtocolId = ProtocolId::Ft4;
}

const FT4_SYNC_BLOCKS: [SyncBlock; 4] = [
    SyncBlock { start_symbol:  0, pattern: &[0, 1, 3, 2] },
    SyncBlock { start_symbol: 33, pattern: &[1, 0, 2, 3] },
    SyncBlock { start_symbol: 66, pattern: &[2, 3, 1, 0] },
    SyncBlock { start_symbol: 99, pattern: &[3, 2, 0, 1] },
];
```

**WSPR** — structurally different on all three axes. The `Fec` and
`Msg` associated types switch to a new pair, and the sync is
expressed via `SyncMode::Interleaved`:

```rust
use mfsk_core::fec::conv::ConvFano;
use mfsk_core::msg::wspr::Wspr50Message;

pub struct Wspr;

impl ModulationParams for Wspr {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 8192;                  // ~683 ms @ 12 kHz
    const TONE_SPACING_HZ: f32 = 12_000.0 / 8192.0;  // ≈ 1.4648
    const GRAY_MAP: &'static [u8] = &[0, 1, 2, 3];
    // …
}

impl FrameLayout for Wspr {
    const N_DATA: u32 = 162;
    const N_SYNC: u32 = 0;                   // sync is embedded in data symbols
    const N_SYMBOLS: u32 = 162;
    const SYNC_MODE: SyncMode = SyncMode::Interleaved {
        sync_bit_pos: 0,                     // LSB of the tone index
        vector: &WSPR_SYNC_VECTOR,           // 162-bit npr3
    };
    const T_SLOT_S: f32 = 120.0;
    const TX_START_OFFSET_S: f32 = 1.0;
}

impl Protocol for Wspr {
    type Fec = ConvFano;                     // convolutional + Fano
    type Msg = Wspr50Message;                // 50-bit message
    const ID: ProtocolId = ProtocolId::Wspr;
}
```

Calling code just passes the ZST as a type argument —
`decode_frame::<Ft4>(...)` or the WSPR-specific
`wspr::decode::decode_scan_default(...)` — and the trait composition
pulls in the appropriate FEC, message codec, and sync mode
automatically.

### Monomorphisation & zero cost

All hot-path functions (`core::sync::coarse_sync::<P>`,
`core::llr::compute_llr::<P>`,
`core::pipeline::process_candidate_basic::<P>`, …) take
`P: Protocol` as a **compile-time** type parameter. rustc
monomorphises one copy per concrete protocol; LLVM sees a
fully-specialised function and inlines the trait constants as
literals. The abstraction is free — the generated FT8 code is
byte-identical to the hand-written FT8-only path the library was
forked from, and FT4 benefits from every micro-optimisation applied
to the shared functions.

`dyn Trait` is reserved for cold paths only: the FFI boundary, the
protocol toggle in JS, and the `MessageCodec` that unpacks decoded
text (which runs once per successful decode, not once per candidate).

### Adding a new protocol

How much work a new protocol needs depends on how much of the
existing infrastructure it can reuse.

1. **Same FEC and same message as an existing mode** (e.g. FT2, or
   the other FST4 sub-modes). Define a new ZST and swap the numeric
   constants (`NTONES`, `NSPS`, `TONE_SPACING_HZ`, `SYNC_MODE`, and
   the sync pattern). `Fec` and `Msg` can be type aliases to the
   existing implementations, and the full `decode_frame::<P>()`
   pipeline runs unchanged.

2. **New FEC but same message** (e.g. a different LDPC size). Add
   the codec as a new module under `fec/` and implement `FecCodec`
   for it. The BP / OSD / systematic-encode algorithms generalise
   naturally across LDPC sizes, so the only real changes are the
   parity-check and generator tables and the code dimensions (N, K).
   `fec::ldpc240_101` is the concrete example to follow.

3. **Both FEC and message are new** (e.g. WSPR). Add the FEC
   implementation, add the message codec, and — if the sync
   structure is fundamentally different — extend `SyncMode` with a
   new variant. WSPR was added via this route, introducing
   `ConvFano` + `Wspr50Message` + `SyncMode::Interleaved` while
   continuing to use the existing pipeline machinery (coarse
   search, spectrogram, candidate de-duplication, CRC check,
   message unpack).

## 3. Shared primitives (`core`)

### DSP (`mfsk_core::core::dsp`)

| Module           | Purpose                                                     |
|------------------|-------------------------------------------------------------|
| `resample`       | linear resampler to 12 kHz                                  |
| `downsample`     | FFT-based complex decimation (`DownsampleCfg`)              |
| `gfsk`           | GFSK tone-to-PCM synthesiser (`GfskCfg`)                    |
| `subtract`       | phase-continuous least-squares SIC (`SubtractCfg`)          |

Each takes a runtime `*Cfg` struct (not `<P>`) because the tuning
parameters include composite-FFT sizes that are not trivially derived
from trait constants alone. Protocol modules expose module-level
constants for each — `ft8::downsample::FT8_CFG`,
`ft4::decode::FT4_DOWNSAMPLE`, etc.

### Sync (`mfsk_core::core::sync`)

* `coarse_sync::<P>(audio, freq_min, freq_max, …)` — UTC-aligned 2D
  peak search over `P::SYNC_MODE.blocks()`.
* `refine_candidate::<P>(cd0, cand, search_steps)` — integer-sample
  scan + parabolic sub-sample interpolation.
* `make_costas_ref(pattern, ds_spb)` / `score_costas_block(...)` — raw
  correlation helpers exposed for diagnostics and custom pipelines.

### LLR (`mfsk_core::core::llr`)

* `symbol_spectra::<P>(cd0, i_start)` — per-symbol FFT bins.
* `compute_llr::<P>(cs)` — four WSJT-style LLR variants (a/b/c/d).
* `sync_quality::<P>(cs)` — hard-decision sync symbol count.

### Equalise (`mfsk_core::core::equalize`)

* `equalize_local::<P>(cs)` — per-tone Wiener equaliser driven by
  `P::SYNC_MODE.blocks()` pilot observations; linearly extrapolates any tones
  that Costas doesn't visit.

### Pipeline (`mfsk_core::core::pipeline`)

* `decode_frame::<P>(...)` — coarse sync → parallel process_candidate → dedupe.
* `decode_frame_subtract::<P>(...)` — 3-pass SIC driver.
* `process_candidate_basic::<P>(...)` — single-candidate BP+OSD.

AP-aware variants live in `msg::pipeline_ap` because AP hint
construction is 77-bit specific.

## 4. Feature flags

| Feature       | Default | Effect                                                        |
|---------------|---------|---------------------------------------------------------------|
| `ft8`         | on      | FT8 ZST, decode, wave_gen                                    |
| `ft4`         | on      | FT4 ZST, decode                                              |
| `fst4`        | off     | FST4-60A ZST, decode                                         |
| `wspr`        | off     | WSPR ZST, decode, synth, spectrogram search                  |
| `jt9`         | off     | JT9 ZST, decode                                              |
| `jt65`        | off     | JT65 ZST, decode (+ erasure-aware RS)                        |
| `full`        | off     | Aggregate of all six protocols                                |
| `parallel`    | on      | Enables rayon `par_iter` in pipeline (no-op under WASM)       |
| `osd-deep`    | off     | Adds OSD-3 fallback to AP decodes under ≥55-bit lock          |
| `eq-fallback` | off     | Lets `EqMode::Adaptive` fall back to non-EQ when EQ fails     |

Both `osd-deep` and `eq-fallback` are heavy: they were measured to
boost FT4's −18 dB success rate by ~5/10 → 6/10 at the cost of ~10×
decode time. Left **off** by default so the stock build fits a 7.5 s
WASM slot comfortably; turn them on when running on a desktop where
CPU budget is abundant.

## 5. Using from Rust

### 5.1 Dependencies

```toml
[dependencies]
mfsk-core = { version = "0.1", features = ["ft8", "ft4", "wspr"] }
```

Pull in only the protocol features you need; the examples below
enable several for illustration.

### 5.2 FT8 decode — minimal example

```rust
use mfsk_core::ft8::{
    decode::{decode_frame, DecodeDepth},
    wave_gen::{message_to_tones, tones_to_i16},
};
use mfsk_core::msg::wsjt77::{pack77, unpack77};

// 1. Synthesise an FT8 frame and pad it into a 15-second slot.
let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones(&msg77);
let frame = tones_to_i16(&tones, /* freq */ 1500.0, /* amp */ 20_000);

let mut audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let start = (0.5 * 12_000.0) as usize;
for (i, &s) in frame.iter().enumerate() {
    if start + i < audio.len() { audio[start + i] = s; }
}

// 2. Decode it back.
for r in decode_frame(&audio, 100.0, 3_000.0, 1.0, None,
                      DecodeDepth::BpAllOsd, 50) {
    if let Some(text) = unpack77(&r.message77) {
        println!("{:7.1} Hz  dt={:+.2} s  SNR={:+.0} dB  {}",
                 r.freq_hz, r.dt_sec, r.snr_db, text);
    }
}
```

### 5.3 WSPR — a separate demod path that still fits the abstraction

WSPR takes symbol-length FFTs directly at 12 kHz rather than
decimating to an FT-style baseband first, so its demodulation
pipeline is staged differently. The `wspr` module exposes its own
entry points. The FEC (`ConvFano`) and message codec
(`Wspr50Message`) are still declared as associated types on
`impl Protocol for Wspr`, so the trait surface remains consistent
— only the slot-level decoder differs.

```rust
use mfsk_core::wspr::decode::decode_scan_default;
use mfsk_core::msg::WsprMessage;

let samples_f32: Vec<f32> = /* 120 s × 12 kHz of f32 samples */;

let decodes = decode_scan_default(&samples_f32, /*sample_rate*/ 12_000);
for d in decodes {
    match d.message {
        WsprMessage::Type1 { callsign, grid, power_dbm } => {
            println!("{:7.2} Hz  {} {} {}dBm", d.freq_hz, callsign, grid, power_dbm);
        }
        WsprMessage::Type2 { callsign, power_dbm } => {
            println!("{:7.2} Hz  {} {}dBm", d.freq_hz, callsign, power_dbm);
        }
        WsprMessage::Type3 { callsign_hash, grid6, power_dbm } => {
            println!("{:7.2} Hz  <#{:05x}> {} {}dBm",
                     d.freq_hz, callsign_hash, grid6, power_dbm);
        }
    }
}
```

`decode_scan_default` runs the (frequency × time) coarse search over
the whole slot internally. If the frequency and start sample are
already known, `wspr::decode::decode_at(samples, rate,
start_sample, freq_hz)` bypasses the scan.

### 5.4 Sniper mode + AP hint

Narrowing the search to ±500 Hz and supplying an a-priori hint lets
the decoder recover weaker signals:

```rust
use mfsk_core::ft8::decode::{decode_sniper_ap, DecodeDepth, ApHint};
use mfsk_core::core::equalize::EqMode;

let ap = ApHint::new().with_call1("CQ").with_call2("JA1ABC");
for r in decode_sniper_ap(&audio, /*target_hz*/ 1000.0,
                          DecodeDepth::BpAllOsd, /*max_cand*/ 15,
                          EqMode::Adaptive, Some(&ap)) {
    // …
}
```

The FT4 equivalent is `ft4::decode::decode_sniper_ap` with a similar
signature.

### 5.5 JT9 / JT65

Both JT9 and JT65 expose the same scan + point-decode pattern:

```rust
use mfsk_core::jt65::decode_scan_default;

let audio_f32: Vec<f32> = /* 60 s × 12 kHz of f32 samples */;
for d in decode_scan_default(&audio_f32, 12_000) {
    println!("{:7.2} Hz  {}", d.freq_hz, d.message);
}
```

JT65 additionally offers `decode_at_with_erasures` for low-SNR
signals where RS erasure decoding can recover frames that the
standard decoder misses.

## 6. C / C++ consumers via `mfsk-ffi`

### Artefacts

`cargo build -p mfsk-ffi --release` emits:

* `target/release/libmfsk.so`  (Linux / Android shared object)
* `target/release/libmfsk.a`   (static, for bundling)
* `mfsk-ffi/include/mfsk.h`    (cbindgen-generated, committed)

### API

See `mfsk-ffi/include/mfsk.h` for the authoritative declarations.
Summary:

```c
enum MfskProtocol {
    MFSK_PROTOCOL_FT8     = 0,
    MFSK_PROTOCOL_FT4     = 1,
    MFSK_PROTOCOL_WSPR    = 2,
    MFSK_PROTOCOL_JT9     = 3,
    MFSK_PROTOCOL_JT65    = 4,
    MFSK_PROTOCOL_FST4S60 = 5,
};

uint32_t          mfsk_version(void);           // major<<16 | minor<<8 | patch
MfskDecoder*      mfsk_decoder_new(MfskProtocol protocol);
void              mfsk_decoder_free(MfskDecoder* dec);

MfskStatus        mfsk_decode_i16(MfskDecoder*, const int16_t* samples,
                                  size_t n, uint32_t sample_rate,
                                  MfskMessageList* out);
MfskStatus        mfsk_decode_f32(MfskDecoder*, const float*,  size_t,
                                  uint32_t, MfskMessageList* out);

MfskStatus        mfsk_encode_ft8(const char* call1, const char* call2,
                                  const char* report, float freq_hz,
                                  MfskSamples* out);
MfskStatus        mfsk_encode_ft4(...);      // same shape
MfskStatus        mfsk_encode_fst4s60(...);  // same shape
MfskStatus        mfsk_encode_wspr(const char* call, const char* grid,
                                   int32_t power_dbm, float freq_hz,
                                   MfskSamples* out);
MfskStatus        mfsk_encode_jt9(...);      // same shape as ft8
MfskStatus        mfsk_encode_jt65(...);     // same shape as ft8

void              mfsk_message_list_free(MfskMessageList* list);
void              mfsk_samples_free(MfskSamples* s);
const char*       mfsk_last_error(void);
```

`MfskMessageList` is caller-owned storage filled by the decode call;
text fields are `char*` UTF-8 NUL-terminated, owned by the list and
freed by `mfsk_message_list_free`.

`MfskSamples` is caller-owned storage filled by encode calls; it
holds 12 kHz f32 PCM and is freed by `mfsk_samples_free`.

See `mfsk-ffi/examples/cpp_smoke/` for a minimal end-to-end demo.

### Memory rules

1. **Handles**: allocate with `mfsk_decoder_new`, free with
   `mfsk_decoder_free`. One handle per thread. Free is idempotent on
   NULL.
2. **Message lists**: zero-initialise a `MfskMessageList` on the
   stack, pass its address to the decode call, free with
   `mfsk_message_list_free` when done reading. Do *not* free
   individual `text` pointers yourself.
3. **Sample buffers**: zero-initialise `MfskSamples`, pass to
   encode, free with `mfsk_samples_free`.
4. **Errors**: on non-zero `MfskStatus`, call `mfsk_last_error` on the
   **same thread** to retrieve a human-readable diagnostic. The
   returned pointer is valid until the next fallible call on that
   thread.

### Thread safety

* An `MfskDecoder` is `!Sync`: one handle per concurrent thread.
* The decoder uses thread-local state for caching and error reporting,
  so spawning multiple threads each with its own handle is cheap.

## 7. Kotlin / Android consumers

`mfsk-ffi/examples/kotlin_jni/` ships a drop-in scaffold:

```kotlin
package io.github.mfskcore

Mfsk.open(Mfsk.Protocol.FT4).use { dec ->
    val pcm: ShortArray = /* captured audio */
    for (m in dec.decode(pcm, sampleRate = 12_000)) {
        Log.i("ft4", "${m.freqHz} Hz  ${m.snrDb} dB  ${m.text}")
    }
}
```

* `libmfsk.so` built via `cargo build --target aarch64-linux-android -p mfsk-ffi`.
* `libmfsk_jni.so` built from the ~115-line C shim, marshals
  `ShortArray` ↔ `MfskMessageList`.
* `Mfsk.kt` exposes an `AutoCloseable` Kotlin class; use with
  `.use { }` to guarantee release.

Full build instructions in `mfsk-ffi/examples/kotlin_jni/README.md`.

## 8. Protocol notes

| Protocol   | Slot   | Tones | Symbols | Tone Δf    | FEC              | Msg   | Sync       | Status |
|------------|--------|-------|---------|------------|------------------|-------|------------|--------|
| FT8        | 15 s   | 8     | 79      | 6.25 Hz    | LDPC(174, 91)    | 77 b  | 3×Costas-7 | implemented |
| FT4        | 7.5 s  | 4     | 103     | 20.833 Hz  | LDPC(174, 91)    | 77 b  | 4×Costas-4 | implemented |
| FST4-60A   | 60 s   | 4     | 160     | 3.125 Hz   | LDPC(240, 101)   | 77 b  | 5×Costas-8 | implemented |
| FST4 other | 15–1800 s | 4 | var     | var        | LDPC(240, 101)   | 77 b  | 5×Costas-8 | one more ZST per sub-mode |
| WSPR       | 120 s  | 4     | 162     | 1.465 Hz   | conv r=½ K=32 + Fano | 50 b | per-symbol LSB (npr3) | implemented |
| JT9        | 60 s   | 9     | 85      | 1.736 Hz   | conv r=½ K=32 + Fano | 72 b  | 16 distributed | implemented |
| JT65       | 60 s   | 65    | 126     | 2.69 Hz    | RS(63, 12) GF(2⁶)     | 72 b  | 63 distributed | implemented |

FST4 does not share FT8's LDPC(174, 91); it uses a separate
LDPC(240, 101) + 24-bit CRC, implemented as `fec::ldpc240_101`.
The BP / OSD algorithms are structurally the same across LDPC
sizes, so the new material is essentially the parity-check and
generator tables together with the code dimensions. FST4-60A is
complete end-to-end; the other FST4 sub-modes (-15/-30/-120/-300/
-900/-1800) differ only in `NSPS` / `SYMBOL_DT` /
`TONE_SPACING_HZ`, and each can be added as a short ZST reusing the
same FEC, sync and DSP.

WSPR is structurally different from the FT modes: it uses
convolutional coding (`fec::conv::ConvFano`, ported from WSJT-X
`lib/wsprd/fano.c`) rather than LDPC, a 50-bit message rather than
77-bit (`msg::wspr::Wspr50Message`, covering Types 1 / 2 / 3), and
a per-symbol interleaved sync (`SyncMode::Interleaved`) rather than
block Costas arrays. The `wspr` module contributes its own TX
synthesiser, RX demodulator, and a quarter-symbol spectrogram used
to keep the coarse search over a 120-s slot within a reasonable time
budget.

JT9 reuses the convolutional FEC (`ConvFano`) and a 72-bit JT
message codec (`msg::jt72::Jt72Codec`). JT65 uses Reed-Solomon
`fec::rs63_12::Rs63_12` with erasure-aware decoding based on Karn's
Berlekamp-Massey algorithm.

## License

Library code is GPL-3.0-or-later, derived from WSJT-X reference
algorithms.
