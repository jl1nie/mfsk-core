# embedded-shared — agent notes

`no_std` crate factored out of the bench / app crates under
`embedded-poc/`. Currently holds:

- streaming-pipeline scaffolding (per-half stage1 incremental
  FFT feed, dispatch helpers),
- 48 kHz / 16 kHz → 12 kHz Q32 linear resamplers reused across
  WAV-sim, UAC, and FFI streaming surfaces,
- `BASIS` scratch placement helpers (heap-allocated since Phase
  0.7a, see commit `6e8f934`),
- `compute_bench` glue that drives `decode_block` against
  canned WAV inputs for the bench crates.

See [`embedded-poc/CLAUDE.md`](../CLAUDE.md) for the shared
toolchain setup (same `esp` toolchain, same `~/export-esp.sh`
gate; targets `xtensa-esp32-espidf` and `xtensa-esp32s3-espidf`
in turn depending on which downstream crate is building).

## Crate-specific notes

- **`no_std + alloc`** — same constraints as the downstream
  bench / app crates. Do not pull `std`-only deps.
- **Streaming API** is the seam the m5stack-s3-app Phase 1 UAC
  port hooks into; see `docs/notes/ROADMAP.md` Phase B and the
  `mfsk_ft8_stream_*` ABI in `mfsk-ffi-ft8/src/stream.rs`.
- **Diagnostics**: `compute_bench` writes per-half timing to
  the logger configured by the downstream crate.
