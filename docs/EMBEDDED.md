# Embedded crates — overview

`mfsk-core` (the host-and-embedded-shared FT8 decoder library
published on crates.io) ships with a set of in-tree embedded crates
under `embedded-poc/` that integrate it with real Xtensa ESP32
hardware. This document is the **entry point for understanding the
embedded story**: which boards are supported, what each crate does,
and where the operational manuals live.

For host-side library usage (no embedded), see
[`docs/LIBRARY.md`](LIBRARY.md).

---

## Boards and roles

| Board | Status | Role |
|---|---|---|
| **M5StickS3** (ESP32-S3 LX7) | ✅ Production | Handheld FT8 controller — acoustic mic, BLE CI-V to IC-705, LCD UI, QSO FSM, optional WiFi UDP log. **Daily-use target**. |
| **M5Stack Core2** (ESP32 classic LX6) | 🔬 Diagnostic / second-board verifier | Decoder runs on `wav_sim` baked-WAV loop; LCD shows results. External I/O (mic, speaker, BLE) **deferred — hardware spec TBD**. Exists to cross-validate the `mfsk-app-shared` board-agnostic API on LX6. |
| **M5Stack CoreS3** (ESP32-S3 LX7) | 📋 Planned | Successor to S3 for the **main UAC controller target** — CoreS3 has the VBUS source + AW9523B BUS_OUT_EN circuitry that M5StickS3 lacks, so true USB-Host audio class to IC-705 will land here. Hardware not yet acquired. |
| **M5StickS3 compute bench** (ESP32-S3) | 🛠️ Tool | Decoder-only benchmark crate (`m5stack-s3/`), drives `decode_block` against canned WAV inputs for timing sweeps. Not for end users. |

---

## Crate layout under `embedded-poc/`

```
embedded-poc/
├── CLAUDE.md                   ← cross-board agent / toolchain notes
├── scripts/
│   ├── flash-monitor.sh        ← canonical flash + capture wrapper
│   └── udp-log-listen.sh       ← PC-side UDP log receiver
├── assets/                     ← vendored WSJT-X reference WAVs
├── m5stack-s3-app/             ← M5StickS3 FT8 controller (PRODUCTION)
│   ├── MANUAL.md / MANUAL.ja.md   ← user manual (start here)
│   ├── cfg-sample.toml         ← WiFi / station config template
│   └── src/
├── m5stack-core2-app/          ← Core2 wav_sim decoder (DIAGNOSTIC)
│   └── cfg-sample.toml
├── m5stack-cores3-app/         ← (planned) CoreS3 main UAC target
├── m5stack-s3/                 ← S3 compute bench (DECODER PROFILING)
├── mfsk-app-shared/            ← board-agnostic app logic (QSO / UI / WiFi / log)
├── embedded-shared/            ← no_std decoder integration (FFT planner / dual_core / stage1_inc)
└── idf-component/              ← esp-idf component shim wrapping mfsk-ffi-ft8 for C-only ESP-IDF projects
```

### Two shared layers

There are two **shared crates** between the app crates, at different
abstraction levels:

- **`embedded-shared/`** — decoder-level. `no_std`, holds the
  streaming pipeline (`stage1_inc` incremental FFT spectrogram
  builder), the dual-core dispatcher, the esp-dsp FFT planner
  bridge, the I2S resamplers, and the per-board sample feed glues
  (`wav_sim`, `compute_bench`). Anything that touches the decoder
  kernel itself.
- **`mfsk-app-shared/`** — controller-level. The QSO state machine,
  UI primitives + `embedded-graphics` draw routines, WiFi STA + UDP
  log datagram sink, ADIF + flash log placeholders, NVS-backed
  `BootMode`, the `LogFanout` log multiplexer. Anything above the
  decoder.

The boundary is **data flow** (channels + shared state mutex), not
callbacks — no traits cross the boundary. Both crates are consumed
by every app crate (s3-app, core2-app, future cores3-app).

---

## For end users: how to operate the firmware

Start with the per-board manual. Today only one exists:

- **M5StickS3**: [`embedded-poc/m5stack-s3-app/MANUAL.md`](../embedded-poc/m5stack-s3-app/MANUAL.md)
  ([JA](../embedded-poc/m5stack-s3-app/MANUAL.ja.md))

The S3 manual covers:

1. The seven boot modes and the two daily-use ones (Acoustic, Qso).
2. Hardware required + optional.
3. First-time setup: toolchain, build, flash.
4. `cfg.toml` reference (WiFi credentials, operator callsign / grid).
5. `BootMode` switching via KEY1 boot-hold or KEY2 long-press.
6. The post-Phase-1.7.7 decode pipeline architecture (Goertzel
   per-symbol DFT, zero internal-DRAM scratch).
7. UI / button reference / menu.
8. QSO workflow.
9. Troubleshooting (USB-CDC freeze, OOM, BLE pairing, …).

Core2 and CoreS3 user manuals will be added when their external I/O
hardware is decided.

---

## For library consumers: what `mfsk-core` asks of embedded callers

`mfsk-core` is `no_std + alloc` capable. The FT8 decode path
(`mfsk_core::ft8::decode_block`) runs on chips with as little as
~150 KB of usable RAM, given:

- A complex single-precision FFT backend (`FftPlanner` /
  `FftPlanner16` trait pair). Embedded targets enable the
  `fft-extern` feature and provide a factory function the linker
  picks up; the in-tree `embedded-poc/embedded-shared/esp_dsp_fft.rs`
  is the reference Xtensa LX6/LX7 binding to `esp-dsp`'s
  `dsps_fft2r_*` family.
- (Optional) A Q15 dot product (`mfsk_core_dot_q15_i32` extern) for
  the BASIS path. **Post-Phase 1.7.7-Stick** this is no longer used
  for new code — the per-symbol DFT now runs through the Goertzel
  recursion in `fill_symbol_spectra_goertzel`, which needs no
  external symbols. The BASIS `dot_q15_i32` path remains for API
  back-compat until the 0.7.0 cleanup.

### Cargo feature combinations

The two embedded-relevant features are:

- **`fft-extern`** — enables the extern-factory FFT-planner contract.
  Embedded crates pair this with their own `mfsk_core_make_default_fft_planner*`
  implementation.
- **`fixed-point`** — switches the decoder's spec / cs path to the
  i16 / Q15 scalar variants instead of f32. Embedded targets enable
  this for halved PSRAM bandwidth and a smaller BP scratch
  (~6 KB instead of ~12 KB).

**Note (added in 0.6.4)**: `fixed-point` now implies `nstep-half`
(NSTEP = NSPS/2 = 960 samples per spectrogram column, vs the host
default NSPS/4 = 480). The two features were independent for
historical reasons but were always co-enabled on every embedded
target. Decoupling silently gave host fixed-point a different
time-grid than embedded, making host validation tests diverge from
embedded behaviour. Coupling them ensures host fixed-point builds
faithfully simulate the embedded path.

### Scalar variants

The whole DSP / FEC pipeline is parameterised by **scalar traits**, so
the same source compiles to either a host-friendly f32 path or an
embedded-friendly integer path with no duplicated code:

- `core::scalar::SpecScalar` — spectrogram / DFT-output scalar
  (f32 on host, `i16` Q14 on embedded when `fixed-point` is on).
- `core::scalar::LlrT` — LLR scalar (f32 on host, Q3i8 on embedded).
- `core::scalar::CoarseAcc` — coarse-sync accumulator (f32; integer
  variant was retired post-0.5).

The trait selection happens entirely through `#[cfg(feature = …)]`
type aliases; there is **no runtime dispatch**.

---

## For developers: where to dig in

### Architecture by layer

| Layer | Crate | Headline files |
|---|---|---|
| Decoder kernel (host + embedded) | `mfsk-core` | `src/ft8/decode_block/{spectrogram, coarse_sync, fill_symbol_spectra, process_candidates}.rs` |
| Streaming pipeline + dual-core | `embedded-poc/embedded-shared` | `src/stage1_inc.rs`, `src/dual_core.rs`, `src/esp_dsp_fft.rs` |
| QSO FSM + UI + WiFi | `embedded-poc/mfsk-app-shared` | `src/qso.rs`, `src/ui/`, `src/wifi.rs`, `src/log_sink.rs`, `src/boot_mode.rs` |
| Board glue (I2S, BLE, LCD, PMIC) | per-app crate | `src/{audio, civ, display, pmic, board}.rs` |

### Toolchain bring-up

See [`embedded-poc/CLAUDE.md`](../embedded-poc/CLAUDE.md) for the
shared `espup install` / `~/export-esp.sh` setup, the
flash-and-capture template, and the LX6 vs LX7 (vs LX7+PIE)
comparison table.

### Why Goertzel (Phase 1.7.7-Stick design rationale)

The per-symbol DFT for FT8 needs to evaluate `Σ x[n] exp(-jωn)` at
each tone frequency across 1920 samples, for every (symbol, tone)
combination in every candidate's cs matrix. The original embedded
path used a precomputed Q15 sin/cos table (`BASIS`) and an asm Q15
dot product — fast (1 cycle/sample via PIE on LX7), but the table
needed 60 KB of internal DRAM × 2 (re/im) × 2 (dual_core main +
worker) = 120 KB.

That 120 KB of internal DRAM is exactly what the I2S bidir DMA
descriptor needs for Qso mode on M5StickS3. The board's free
contiguous internal chunk was too small to satisfy both, and Qso
mode boot failed.

Phase 1.7.7 replaced the BASIS dot-product with a generalised
Goertzel recursion: per (sym, tone), a 2-tap IIR with 3 f32 state
values that get thrown away on return. Zero internal-DRAM scratch.

The performance trick: order the loop **sample-outer / tone-inner**
so the 8 per-tone recursions (one per FT8 tone) run as 8
independent dependent chains through the FPU pipeline. LLVM unrolls
the constant-bound `NTONES = 8` inner loop and the Xtensa FPU
absorbs ~all of the per-chain latency in parallel. Result: stage3
cost matches the BASIS asm dot product (~1.4 s on S3
`qso3_busy.wav`) with **zero scratch + +0.16..+0.63 dB SNR
improvement** (f32 Goertzel has more precision than the prior Q15
BASIS dot product).

The full migration log is in the project memory `phase177-goertzel`
and PR #103.

---

## Release status

See [`docs/ROADMAP.md`](ROADMAP.md) for the phase-by-phase plan
across all embedded crates. As of 2026-05-18:

- **M5StickS3 (`m5stack-s3-app`)**: Phase 1.5 → 1.7.7-Stick shipped
  (acoustic capture, BLE CI-V, QSO FSM, Goertzel migration).
  Phase 2 (TX scheduler hardening) and Phase 3 (UI polish) in
  progress.
- **M5Stack Core2 (`m5stack-core2-app`)**: Phase 2 scaffold (boots,
  decodes wav_sim, renders to LCD corner crop). External I/O
  deferred pending hardware spec.
- **M5Stack CoreS3 (`m5stack-cores3-app`)**: not yet created. Phase
  0-Core (crate skeleton + UAC port from S3 sibling) pending
  hardware arrival.

`mfsk-core` itself is on crates.io at `0.6.3`; the Phase 1.7.7
features (Goertzel migration, `fixed-point` → `nstep-half` coupling,
mag² saturation fix) land in `0.6.4` (released after PR #103 merges
into `main`).
