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

- **Unit tests** live next to the code they cover.
- **Integration tests** in `mfsk-core/tests/` cover protocol-level
  behaviour (synth → decode round-trips, SNR sweeps, timing budget).
- **FFI smoke tests** live in `mfsk-ffi/examples/cpp_smoke/`; they
  exercise every protocol through the C ABI and include
  multi-threaded stress tests.
- CI runs everything on each push. Keeping the full suite green is
  the baseline expectation for a merge.

## Commit messages

Conventional-ish: a short imperative subject ("fix: FST4 sync pattern
off-by-one"), empty line, and a paragraph or two explaining the *why*
and any cross-references to WSJT-X source. Include the upstream
SHA / file:line if you pulled an algorithmic change from there.

## License

Contributions are GPL-3.0-or-later, matching the crate and WSJT-X
upstream.
