# Feature flags and local checks

Moved out of the repo-root `CLAUDE.md` (2026-10-12): the feature flags that bite, and the two git hooks.

## Feature flags — the ones that bite

`default = ["std", "ft8", "ft4", "parallel", "fft-rustfft"]`;
`full` is every protocol + `uvpacket` + `packet-bytes` + `serde` +
`parallel` + host FFT. The full table with rationale is in
`mfsk-core/Cargo.toml`, which is worth reading as documentation — each
flag carries the measurement that justified it. The traps:

- **`fixed-point` implies `nstep-half`, and must.** Decoupled, host
  fixed-point ran a different time grid (NSTEP=NSPS/4) from embedded
  (NSPS/2) and produced a completely different candidate ranking on the
  same WAV — 4 vs 7 decodes on `qso3_busy` single-pass. The point of host
  fixed-point is to simulate embedded faithfully.
- **`wspr-ddc` and `wspr-ddc-cascade` are mutually exclusive** —
  `compile_error!` in `decode_scan_inner`. They swap WSPR's channelizer
  from the reference whole-slot FFT to the streaming down-converter
  (single-stage / two-stage cascade). Host default stays on the exact
  reference.
- **`internal-testing` is not optional for whole-crate commands.**
  Without it, `cargo clippy --all-targets --features full` reports `E0603`
  private-item errors in `fst4_sweep` / `ft4_sweep` /
  `fst4_wsjtx_samples`. That's a missing flag, not a regression you
  introduced — use the hook's own invocation before concluding otherwise.
- **`jt9` / `jt65` / `q65` stopped being host-only in #390** — they no
  longer force `fft-rustfft` (and with it `std`). Their FFTs go through
  `engine::fft`, the modules carry `alloc::` imports and
  `num_traits::Float` instead of `std`, and JT9's `downsam9` normalises
  its inverse explicitly instead of assuming rustfft's unscaled
  convention. Each has an `alloc,<mode>,fft-extern` row in the
  pre-push/CI feature matrix, which is the only build that catches a
  `std::` path creeping back. Compiling is all that changed: no
  fixed-point path and no board app drives them. `fst4` got there first
  by a shorter route (issue #306) — it never referenced `rustfft` at
  all.
- **`jtty` and `msk144` are features with no ZST.** Neither is in
  `PROTOCOLS`. `jtty`'s wire level (source, CRC-12, tbcc, tx, pack) needs
  no FFT, while `jtty::rx` / `dsp` / `assemble` compile only with
  `fft-rustfft` or `fft-extern`; the matrix has `jtty`, `alloc jtty` and
  `alloc jtty fft-extern` rows for exactly that split. `msk144`'s
  burst-scan sync needs an FFT feature too.
- **`uvpacket` declares `std` explicitly** (it reaches for
  `std::f32::consts::PI` and std's prelude). Making it genuinely
  no_std-capable is separate, deliberate work.
- `fft-extern` / `dotprod-extern` are the embedded hooks: the final binary
  supplies `mfsk_core_make_default_fft_planner` / `mfsk_core_dotprod_f32`.
  See `engine::fft`'s docs and `EMBEDDED.md` "The FFT extern Rust
  contract".
- `wspr-pass2-topn` and `wspr-fano-cap-fast` are embedded speed/floor
  trades, off on host so host stays `wsprd`-faithful. `wspr-fano-cap-fast`
  is **not a free knob upward** — raising the cycle budget past the
  reference starts manufacturing phantom decodes (swept table in
  `wspr::decode`).

## Local checks — two hooks, and they are different

There are two, and **one line enables both** — they live side by side in
`.githooks/`. Each catches things the other doesn't:

```sh
git config core.hooksPath .githooks     # pre-commit AND pre-push
```

(`bash scripts/install-hooks.sh` does exactly this and lists what it
enabled. It used to *copy* a pre-push shim into `.git/hooks/` instead,
which git never ran: `core.hooksPath` makes git read hooks from that
directory **instead of** `$GIT_DIR/hooks`, never both. Following the
old two-line instruction therefore left the feature matrix silently
disabled — for however long both were configured, pushes went through
with no matrix and nothing said so. Fixed 2026-08-30 by moving the
pre-push hook into `.githooks/` where the same config line picks it
up.)

- `.githooks/pre-commit` — `cargo fmt --all --check`, `cargo clippy
  --workspace --all-targets --features full,internal-testing --no-deps --
  -D warnings`, and `RUSTDOCFLAGS="-D warnings" cargo doc -p mfsk-core
  --features full --no-deps`. The rustdoc step is there because broken
  intra-doc links block CI's `docs` job and pollute docs.rs.
- `scripts/pre-push-check.sh` — adds `-D clippy::perf` and the
  **feature matrix** (`FEATURE_MATRIX` in that script is the list — read
  it rather than this file: `--no-default-features` alone, each protocol
  feature, the `alloc …` / `fft-extern` / `fixed-point` embedded rows,
  `full`. `ci.yml`'s `feature-matrix` job is the counterpart and is not
  identical: it also builds `msk144` and `wspr wspr-pass2-topn`), then
  `mfsk-ffi`'s tests under both its feature sets and the FT8 fixed-point
  recall floors. This exists because PR #233 shipped code that compiled only under
  `full` — `dead_code` reachability depends on which cfg'd items a given
  feature set exposes, so passing one or two combos proves nothing about
  the rest.

CI (`ci.yml`) then runs: `changes` (a path filter that decides which sweep
suites are relevant), `lint`, `test` (the tier matrix — see below),
`feature-matrix`, `ffi` (`mfsk-ffi` Rust tests under both its feature
sets, the C++ driver including its multi-thread stress, and
`mfsk-app-shared-hosttest`), `cross` (windows-gnu + android arm64,
asserting the DLL exports its symbols and the `.so` is 16 KB aligned),
`docs`, and `publish-dry-run`. `RUSTFLAGS: -D warnings` is set globally.

**The host toolchain is pinned** in the root `rust-toolchain.toml`
(1.98.1 as of 2026-10-02), and every `dtolnay/rust-toolchain@` in
`ci.yml` / `release.yml` names that version — the `lint` job fails if
one doesn't. It was `@stable` until rustc 1.99.0's aarch64
loop-vectorizer hang (rust-lang/rust#163626) held two jobs for six
hours. The `msrv` job overrides it with `RUSTUP_TOOLCHAIN` (a toolchain
file outranks rustup's default). Downstream crates never see the pin,
so `toolchain-canary.yml` builds and tests on the latest stable (beta
weekly) outside the merge gate; raise the pin once that is green on the
newer version. The `esp` toolchains under `embedded-poc/` are separate
and unaffected.

`kotlin` (the JNI shim + a JVM test over `bindings/kotlin/`) runs
per-PR too. **None of these three cost wall clock**: measured at 120 s,
90-140 s and ~70 s against the `Test (tier A+B)` job's ~300 s, which
gates every run anyway. The Kotlin one was briefly moved to release time
on the belief that it was slow — the 72-minute first run turned out to
be a hang in the shim, not a cost, and the measurement put it back.

`swift` (the XCTest suite over `bindings/swift/`, plus the
`aarch64-apple-ios` cross-compile) runs per-PR too, and is **the one
macOS runner in this repo**. It earns the slot twice: XCTest ships with
Xcode rather than with the Command Line Tools, and the iOS SDK is
Xcode's as well — which is why the `cross` job stops at Windows and
Android rather than iOS being missing.

To run them yourself: `bindings/kotlin/build.sh` (needs `JAVA_HOME` and
`kotlinc` on PATH) and `bindings/swift/scripts/test.sh` (needs Xcode;
it will point `DEVELOPER_DIR` at it if `xcode-select` is on the CLT).


