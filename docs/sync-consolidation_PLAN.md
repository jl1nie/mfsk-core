# sync consolidation plan — unify FT8 coarse_sync, then generalise

> Status: design draft. Builds on PR #46 (the minimal #40 fix that
> routes `ft8::sync::coarse_sync` through `decode_block`).

## Problem

There are currently three coarse_sync code paths in the tree:

| Path                                                         | Used by                                                         | Algorithm                                                                                  |
| ------------------------------------------------------------ | --------------------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| `mfsk-core/src/core/sync.rs::coarse_sync<P: Protocol>`       | FT4, FST4, JT9, JT65, Q65, WSPR, uvpacket via per-protocol shim | Same-time-slot non-Costas-tone reference (single NTONES-wide window). Protocol-trait generic. |
| `mfsk-core/src/ft8/sync.rs::coarse_sync` (thin wrapper)      | `ft8::decode::decode_frame*` (host wide-band)                   | After PR #46: forwards to `decode_block::coarse_sync`. Before PR #46: forwarded to `core::sync::coarse_sync<Ft8>`. |
| `mfsk-core/src/ft8/decode_block.rs::coarse_sync` + `coarse_sync_with_allsum` | `decode_block` (host f32 default + embedded fixed-point) + post-PR #46 host wide-band | WSJT-X `sync8.f90`-faithful 16-bin sliding-window allsum noise reference. FT8-specific. |

PR #46 unified FT8 onto the `decode_block` algorithm at the call-site
level, but the `core::sync::coarse_sync<P>` codepath is still the only
one that exists for FT4 / FST4 / JT9 / etc., and the `ft8::sync` wrapper
is now a one-line trampoline whose only job is to compute a Spectrogram
and call decode_block.

This isn't broken — it's _accumulated_. The drift happened because:

1. Initial commit (`3c16984`) split from `webft8`, which had a
   single FT8-specific coarse_sync.
2. A later refactor extracted the protocol-generic
   `core::sync::coarse_sync<P>` so FT4 / FST4 / JT9 could share it.
   Each protocol got a thin shim. Generality won; per-protocol
   accuracy (specifically the WSJT-X-faithful noise estimator) was
   simplified out.
3. The embedded port (`f118a64`, May 2 2026) added
   `decode_block::coarse_sync` because `core::sync` couldn't:
   - the `FftPlanner` trait has no i16 backend (esp-dsp's
     `dsps_fft2r_sc16` needs one)
   - `coarse_sync_with_allsum` + `precompute_coarse_allsum_into`
     allow `stage1_inc` to build the allsum row-by-row as PSRAM
     fills, halving wall-clock on Core2 by hiding the precompute
     under the 15 s capture window
   - the fixed-point variants are gated by `#[cfg(feature = "fixed-point")]`
     and have to coexist with the f32 default

   Re-implementing from `sync8.f90` was the simplest path; that the
   resulting algorithm beats `core::sync<Ft8>` on busy bands was
   accidental and surfaced only when PR #39 added the apon recall
   regression test.

## Goal

Reduce the three paths to as few as possible without losing:

- The `Protocol` trait abstraction that lets FT4 / FST4 / JT9 etc.
  share one implementation.
- The embedded constraints (i16 FFT backend, allsum incremental fill,
  fixed-point feature gating) the `decode_block` path was built to
  satisfy.
- WSJT-X parity for FT8 specifically.

## Options

### A. `Protocol` trait gets an associated `Sync` type

```rust
pub trait Protocol {
    type Sync: ProtocolSync<Self>;
    // ... existing items
}

pub trait ProtocolSync<P: Protocol> {
    fn coarse_sync(audio: &[i16], freq_min: f32, freq_max: f32, ...)
        -> Vec<SyncCandidate>;
}
```

Each protocol provides its own `Sync` impl; the generic free function
just dispatches:

```rust
pub fn coarse_sync<P: Protocol>(audio: &[i16], ...)
    -> Vec<SyncCandidate>
{
    P::Sync::coarse_sync(audio, ...)
}
```

**Pros**: Type system enforces "every protocol owns a sync impl";
duplication can't drift silently again. FT8 plugs in `decode_block`,
FT4/FST4/JT9 keep the current generic body wrapped in a default impl.

**Cons**: Wider trait change. Every `impl Protocol for FtN` site has
to add the associated type. Cross-cutting refactor risk.

### B. Drop `core::sync::coarse_sync<Ft8>` specialisation, keep PR #46 routing

This is what PR #46 already starts. After it lands:

- `ft8::sync::coarse_sync` is a 3-line trampoline into `decode_block`.
- `core::sync::coarse_sync<Ft8>` still compiles but has zero callers.

Follow-up:

1. Delete the `ft8::sync::coarse_sync` wrapper entirely; have
   `decode::decode_frame_inner` call `decode_block::compute_spectrogram`
   + `decode_block::coarse_sync` directly (one extra line per call site,
   two call sites).
2. Promote `decode_block::coarse_sync` from `#[doc(hidden)]` to public
   API — it is now THE FT8 coarse_sync, not a benchmarking aid.
3. Leave `core::sync::coarse_sync<P>` in place for FT4 / FST4 / JT9 /
   etc.; mark FT8 as "do not use this path" via a `where P: Protocol +
   NotFt8` bound or just a doc note.

**Pros**: Smallest follow-up. No trait churn. FT4/FST4/JT9 untouched.

**Cons**: The "FT8 lives in decode_block, everything else in core::sync"
split is still a structural surprise for new readers — they'd expect
`ft8::sync` to be the FT8 sync entry point. Mitigated by doc.

### C. Lift the WSJT-X allsum estimator into `core::sync` and let every
        protocol opt in

Generalise the 16-bin sliding-window allsum noise estimator to
parameterise on `Protocol::SYNC_MODE` (Costas pattern + offsets).
Every protocol that has a WSJT-X reference implementation using a
similar sliding-window noise estimate can opt in via a constant on
the `Protocol` trait (e.g. `const NOISE_ESTIMATE: NoiseEstimate =
NoiseEstimate::Allsum16`).

**Pros**: Single coarse_sync everywhere. FT4 might benefit too (its
WSJT-X reference uses a comparable estimator with NFILT=1400 we just
fixed in #27). Eliminates the embedded vs host divide structurally.

**Cons**: Largest scope. Requires reading WSJT-X `lib/{ft4,fst4,jt9,
q65}_decode.f90` to confirm each one's noise estimator actually
matches the allsum shape. Risk of forcing a one-size-fits-all
algorithm on protocols whose WSJT-X reference uses different shapes.
Embedded constraints (incremental allsum fill, i16 FFT) still need
`Protocol::Sync`-style associated-type plumbing for the embedded
port to stay clean.

## Recommendation

**Sequence: B now, A next, C if needed.**

1. **Land PR #46 first** — close #40, get host wide-band recall on
   par with `decode_block`, make sure CI golden tests confirm no
   regression.

2. **Follow-up PR (call it `refactor(ft8): inline decode_block coarse_sync into decode.rs`)**:
   - Delete `ft8::sync::coarse_sync` wrapper.
   - Update the two call sites in `decode.rs` (lines ~766 and ~1138)
     to call `decode_block::compute_spectrogram` + `decode_block::coarse_sync`
     directly.
   - Promote `decode_block::coarse_sync` and
     `decode_block::compute_spectrogram` out of `#[doc(hidden)]`.
   - Doc note in `core::sync::coarse_sync` that FT8 should not call
     this path.

3. **Then evaluate A** — once B is done and shipped, the surface area
   is small enough to do the trait change without conflating concerns.
   `Protocol::Sync` associated type would make the FT8/non-FT8 split
   type-safe and prevent silent regressions if someone adds a new
   protocol and forgets to wire its sync. Schedule for a 0.6.x
   refactor window.

4. **C only if a measurable win surfaces** — e.g. if FT4 or FST4
   recall measurements show the same busy-band noise mis-estimation
   pattern as FT8 had. Without that data, generalising the allsum
   estimator is speculation.

## Migration steps for B (the immediately actionable piece)

After PR #46 lands:

1. Verify `decode_block::coarse_sync` and `compute_spectrogram` have
   stable enough signatures to be public — no breaking changes
   expected for at least 0.5.x.
2. New PR with the changes outlined in step 2 above. Diff size estimate:
   ~20 lines deleted from `ft8/sync.rs`, ~6 lines added in `decode.rs`,
   doc-comment updates on `decode_block::coarse_sync` and
   `compute_spectrogram`.
3. Update `ROADMAP.md` to reference this PLAN under the post-0.5.12
   refactor section.

## Test strategy

- Existing FT8 host tests (`ft8_qso3_apoff_recall`,
  `ft8_qso3_jtdx_recall`, `ft8_qso3_apon_recall`,
  `ft8_reference_suite_recall`) catch any recall regression introduced
  by routing changes.
- `ft8_decode_block_*` already exercise `decode_block::coarse_sync`
  directly — promoting it to public API doesn't change test coverage,
  it just legitimises the existing tests.
- Adding a new test for the inline path in `decode.rs` is unnecessary
  — the recall tests are end-to-end and would catch any wiring bug.

## Out of scope

- Touching the FT4 / FST4 / JT9 / Q65 / WSPR / uvpacket sync paths.
  They stay on `core::sync::coarse_sync<P>`. Option A or C work would
  visit them, not this immediate refactor.
- Changing the `freq_hint` semantics. Currently `decode_block::coarse_sync`
  drops the hint; PR #46's commit message documents this as a follow-up.
  A real fix would extend `decode_block::coarse_sync` to take an optional
  `freq_hint: Option<f32>` and promote nearby candidates the same way the
  `core::sync` path used to. Easy ~10 line addition; orthogonal to the
  consolidation and tracked separately.
- Embedded fixed-point variants (`compute_spectrogram` `#[cfg(feature
  = "fixed-point")]` branch). They keep their current shape; only the
  default f32 path is touched.

## References

- PR #46 — the immediate #40 fix that motivates this plan.
- Issue #40 — host wide-band coarse-sync misses candidates.
- Commit `f118a64` — embedded migration that introduced
  `decode_block::coarse_sync`.
- Commit `3c16984` — initial split from `webft8`; baseline state of
  `core::sync` and `ft8::sync`.
- WSJT-X `lib/ft8/sync8.f90` — source for the 16-bin allsum noise
  estimator that `decode_block::coarse_sync` ports.
