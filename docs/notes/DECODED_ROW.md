# Design note — `Decoded`, the unified host-UI decode row (0.9)

Status: **landed** (core type + all five per-protocol conversions +
`serde` feature). This note records the decisions and the deferred
follow-ups, as the tracking start-point for the 0.9 "streaming
ergonomics for host UIs" cycle.

## Problem

Every protocol's native decode result is structurally distinct —
different field names, and **four** different message-text paths:

| Result type (native)                    | Modes         | Text path            |
|-----------------------------------------|---------------|----------------------|
| `engine::pipeline::DecodeResult`        | FT8/FT4/FST4  | `unpack77` (fallible)|
| `wspr::WsprResult`                      | WSPR          | `WsprMessage: Display` |
| `q65::Q65Result`                        | Q65           | already a `String`   |
| `jt65::Jt65Result` / `jt9::Jt9Result`   | JT65 / JT9    | `Jt72Message: Display` |

A host UI / spotting uploader / IPC bridge that shows more than one
mode re-writes the same "pull out text + freq + dt + snr" glue per
type, and — critically for the streaming path — the borrowed
`&NativeResult` handed to `.on_result` can't cross a thread boundary,
so the extraction has to happen right there anyway.

## Decision

A single owned row, resolved once at the conversion boundary:

```rust
// mfsk_core::msg::decoded
pub struct Decoded {
    pub text: String,
    pub freq_hz: f32,
    pub dt_sec: f32,
    pub snr_db: f32,
    pub protocol: ProtocolId,
}
```

Locked choices (from the 0.9 planning thread):

1. **Name: `Decoded`.**
2. **Scope: minimal cross-mode intersection only** (option A of the
   three considered — vs. flat `Option` extras, vs. an embedded native
   handle). The mode-specific diagnostics (`sync_score`, `hard_errors`,
   `iterations`, `start_sample`, …) don't generalise into clean shared
   columns, so hoisting them would just reintroduce per-mode branching.
   `Decoded` is **additive**: native results stay, keep their extras,
   and remain `Clone`+`Send` for callers who need that detail across a
   channel (they stream the native result instead).
3. **unpack failure → `None`.** The FT8-family conversion is
   `Option<Decoded>`; a payload that passed CRC but won't unpack (rare)
   is not surfaced as a placeholder row.
4. **`serde` feature, off by default**, folded into `full`. Derives on
   `Decoded` + `ProtocolId`, `no_std`-clean via serde's `alloc`.

## Conversion API

Per-protocol `to_decoded(..)` methods (not one trait) — signatures
differ because the modes genuinely differ, matching the crate's
"free-function / per-protocol method over a forced trait" philosophy
(LIBRARY §0.3):

```rust
impl DecodeResult { // FT8/FT4/FST4 (shared type)
    fn to_decoded(&self, protocol: ProtocolId,
                  hash: Option<&CallsignHashTable>) -> Option<Decoded>;
}
impl WsprResult { fn to_decoded(&self) -> Decoded; }               // infallible
impl Q65Result  { fn to_decoded(&self, sample_rate: u32,
                                nominal_start_sample: usize) -> Decoded; }
impl Jt65Result { fn to_decoded(&self, sample_rate: u32,
                                nominal_start_sample: usize) -> Decoded; }
impl Jt9Result  { fn to_decoded(&self, sample_rate: u32,
                                nominal_start_sample: usize) -> Decoded; }
```

- The FT8-family impl is authored in `msg::decoded` (not `engine`) so it
  can call `unpack77` without `engine` depending on `msg` — the impl
  block lives in `msg`, keeping the crate-wide dependency arrow intact.
- `dt_sec` for Q65/JT65/JT9 is `(start_sample − nominal_start_sample) /
  sample_rate`; `nominal_start_sample` is the anchor the caller already
  passed to `decode_scan` / the decode builder. FT8/WSPR carry `dt_sec`
  directly, so their conversions need no sample-rate argument.

## Deferred follow-ups

- **Streaming-doc example** — simplify the Tokio `.on_result` bridge in
  `docs/reference/STREAMING.md` to stream `Decoded` via `to_decoded(..)`
  instead of a hand-rolled struct (the concrete "UI is easier" payoff).
- **FFI mapping** — `Decoded` is a flat, fixed struct precisely so it
  can map to a C struct in `mfsk-ffi` later; not wired yet.
- **Generic dispatch** — no unifying `trait ToDecoded` for now (a
  GAT-based context type could unify the differing signatures). Add only
  when a caller actually needs to convert generically over `P`.
- **Session / slot-timing abstraction** — item (b) of the 0.9 UI work:
  an object owning audio ingestion + slot alignment + decoder lifecycle,
  so callers stop hand-managing 15 s slot buffers. Separate change.
