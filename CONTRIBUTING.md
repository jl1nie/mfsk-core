# Contributing

Thanks for the interest. `mfsk-core` is a 1:1 Rust port of the WSJT-X
algorithms, so changes are easiest to review when they either (a) stay
close to the upstream Fortran source they mirror, or (b) clearly
explain any deliberate departure.

## Dev setup

After cloning:

```
git config core.hooksPath .githooks
```

This wires `.githooks/pre-commit` (`cargo fmt --check` +
`cargo clippy --workspace --all-targets -- -D warnings`) so your
commits match what CI will check.

The heavy test suite is CI-only to keep the hook fast. Run it locally
before pushing anything non-trivial:

```
cargo test -p mfsk-core --features full --release
```

Runs ~150 tests in under a minute, including end-to-end synth → decode
roundtrips for every protocol.

## Repository layout

```
mfsk-core/              — published library crate (crates.io)
  src/
    core/               — protocol traits, DSP, sync, pipeline
    fec/                — LDPC, convolutional/Fano, Reed-Solomon
    msg/                — 77-bit / 72-bit / 50-bit message codecs
    ft8/ ft4/ fst4/
    wspr/ jt9/ jt65/    — per-protocol modules (feature-gated)
  tests/                — integration tests
mfsk-ffi/               — C ABI wrapper, not published to crates.io
  src/lib.rs            — mfsk_* functions
  include/mfsk.h        — cbindgen-generated, committed
  examples/cpp_smoke/   — runnable C++ driver (part of CI)
.githooks/pre-commit    — local fmt + clippy fence
.github/workflows/
  ci.yml                — lint + test + feature matrix + FFI + docs
  release.yml           — tag-triggered crates.io publish
```

## Relationship to WSJT-X

Every algorithm file in `src/` cites the `lib/*.f90` / `lib/*.c` file
it ports. When changing a decoder internal, the expected workflow is:

1. Read the corresponding WSJT-X source.
2. Make the matching change in Rust.
3. Keep the comment pointers valid — if you restructure, update the
   `Ported from…` note so the next reader can cross-reference.

Algorithmic corrections that *diverge* from WSJT-X (e.g. we implement a
smarter refiner than upstream) are welcome, but call them out in a
comment so readers don't assume the Rust code is still a faithful port.

## Adding a new protocol

The payoff of the trait-based abstraction shows up here. To add, say,
a new LDPC-based mode:

1. Create `src/<mode>/mod.rs` and define a ZST (e.g. `pub struct MyMode;`).
2. `impl ModulationParams for MyMode` — tone count, symbol rate, Gray
   map, GFSK shaping constants.
3. `impl FrameLayout for MyMode` — N_SYMBOLS, N_DATA, N_SYNC, slot
   length, sync-block pattern.
4. `impl Protocol for MyMode` — bind `type Fec = …;` and
   `type Msg = …;` to existing codecs (or add a new FEC under `fec/`).
5. Optionally add `decode.rs` / `encode.rs` thin wrappers that call
   into `crate::core::pipeline::decode_frame::<MyMode>` and the
   synth helpers. The pipeline does not need to change.
6. Feature-gate: `#[cfg(feature = "<mode>")]` in `src/lib.rs` and
   `Cargo.toml`.
7. Add a trait-surface test (modulation / frame constants round-trip
   through the trait) and a synth → decode roundtrip test.

FST4-60A is the most recent example — it was added post-hoc without
modifying any shared pipeline code. Cross-reference its commit when
in doubt.

## Testing philosophy

Tests are organised by **the property they assert**, not by where the
file lives. The previous split (unit / integration / FFI) described
location only, and left the suite overwhelmingly weighted toward one
property — recall — with no vocabulary for the others.

That gap was measurable. In a single session (2026-08-11) seven
defects were found and shipped fixes for; **zero** were caught by the
existing suite. The clearest case: WSPR's recall test reported 8/8 on
the WSJT-X golden while the decoder simultaneously emitted 8 phantom
decodes on the same audio — a 50 % false-decode rate, green the whole
time, because recall only ever asks "did the expected messages come
out?" and never "did anything else?".

### The four tiers

| Tier | Property | When it runs | Corpus |
|---|---|---|---|
| **A — Invariants** | unit tests, `protocol_invariants`, encode→decode roundtrips, bit-exact parity (streaming == batch, no-alloc == `Vec`), TX waveform properties | every PR | none (synthetic) |
| **B — Golden fidelity** | **recall** (floor) + **precision** (phantom ceiling) + **SNR accuracy** vs a reference decoder, on real recordings | every PR | vendored in `embedded-poc/assets/golden/`; **missing = failure** in CI |
| **C — Sensitivity** | AWGN / fading threshold curves | **before a release**, locally | generated, ~17 GB, gitignored |
| **D — none** | print-only probes and diagnostics | — | deleted; a test that cannot fail is not a test |

**Tier B is written through `tests/common/golden.rs::assert_golden`,
which takes recall and precision together.** There is deliberately no
recall-only entry point: adding a golden test for a protocol forces a
decision about its phantom budget. Target `max_extra: 0`; a non-zero
budget is a documented debt, not a default.

Tier B must not silently skip. `tests/common/corpus.rs` panics on a
missing asset when `MFSK_REQUIRE_CORPUS=1`, which CI sets. Before the
recordings were vendored, every WSJT-X-sample test resolved from a
sibling checkout CI does not have, so five protocols' golden tests
skipped and reported success.

Tier C is **not** run by CI — the corpora need WSJT-X's Fortran
simulators built and are far too large. Run it locally before cutting
a release, or when you have changed something that moves sensitivity.

### Which tests to run for a given change

| You changed | Run |
|---|---|
| `src/<proto>/decode*` — candidate selection, acceptance, dedup | that protocol's tier A + B, **especially precision** |
| `src/engine/**` — shared DSP/pipeline | tier A + B for **every** protocol on that path (FT4, FST4 and the generic pipeline all share it) |
| an SNR formula | that protocol's tier B SNR check, against real `jt9`/`wsprd` |
| `src/<proto>/tx.rs`, `engine::dsp::{gfsk,envelope}` | tier A TX waveform tests for **all** protocols |
| FEC / LLR / decoding algorithm | tier A + B, **plus tier C before the next release** — these move the threshold |
| a new protocol | fill every tier A and B cell, precision included |

FFI smoke tests (`mfsk-ffi/examples/cpp_smoke/`) exercise every
protocol through the C ABI, including multi-threaded stress, and run
in CI's `ffi` job.

Keeping tiers A and B green is the baseline expectation for a merge.

## Commit messages

Conventional-ish: a short imperative subject ("fix: FST4 sync pattern
off-by-one"), empty line, and a paragraph or two explaining the *why*
and any cross-references to WSJT-X source. Include the upstream
SHA / file:line if you pulled an algorithmic change from there.

## License

Contributions are GPL-3.0-or-later, matching the crate and WSJT-X
upstream.
