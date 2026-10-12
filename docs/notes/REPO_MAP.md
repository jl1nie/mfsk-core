# Repository map

Moved out of the repo-root `CLAUDE.md` (2026-10-12), which keeps a short table. Everything here was in that file; nothing was dropped.

## Repository map

Host workspace members (`Cargo.toml` `[workspace] members`):

| path | what it is | published? |
|---|---|---|
| `mfsk-core/` | the library — everything below is a consumer of it | **yes**, crates.io |
| `mfsk-ffi/` | C ABI over *all* protocols; `include/mfsk.h` is cbindgen-generated and committed; `examples/cpp_smoke/` is a real C++ driver run by CI | no |
| `mfsk-ffi-abi/` | shared `#[repr(C)]` mode / status / params / row types `mfsk-ffi` re-emits (issue #205) | no |
| `hosttest/mfsk-app-shared/` | runs the host-testable parts of `embedded-poc/mfsk-app-shared` under a normal `cargo test` | no |
| `apps/skimmer/core/`, `apps/skimmer/cli/` | sample app: SpyServer skimmer over `IqReceiver` (`skimmer-core` + the `skimmer` CLI); see `apps/skimmer/README.md`. `mfsk-core` itself has no network code, by decision | no |

Only `mfsk-core` reaches crates.io; the rest are `publish = false`.
`workspace.package.version` in the root `Cargo.toml` is the single source
of truth for all of them — a release bump is one edit, and
`release.yml`'s tag check reads that field.

Deliberately **outside** the workspace (`exclude`), because host `cargo
build` would try to compile them with the stable toolchain and fail:

- `embedded-poc/` — stand-alone Cargo projects on their own toolchains
  (`+esp` Xtensa, etc.), each with a path dep on `mfsk-core`. See
  "Embedded targets" below.
- `bench/wasm/` — a wasm-bindgen harness for `wasm32-unknown-unknown`
  (issue #208); a manual/periodic tool, not CI-gated.
- `wasm-smoke/` — one decode per mode on `wasm32-unknown-unknown`, run in Node by CI's `wasm` job (issue #585). Excluded from
  the workspace like `bench/wasm`, but unlike it, gated: `std` does not imply a clock on that target (#583).
- `apps/skimmer/gui/` — the skimmer's Tauri + Svelte GUI over
  `skimmer-core`; needs a webview (WebView2 / webkit2gtk) and Node, which
  host CI lacks. `skimmer-installers.yml` builds it on Windows and macOS
  (PRs touching `apps/skimmer/**`, and `skimmer-vX.Y.Z` tags, whose
  `skimmer-release.yml` makes a release of its own); locally, Windows or a
  Mac. **The skimmer has its own version (0.9.x is the beta; 1.0.0 is the first with PSK Reporter in production)**, not the workspace's:
  four files carry it and that workflow checks them. The library's
  `vX.Y.Z` releases no longer attach the installers (2026-10-11).

`ci.yml`'s `paths` trigger excludes these trees, so a change confined to
them runs no host CI at all. That is intentional, and it is also why "CI
is green" says nothing about an embedded change. Three things under
`embedded-poc/` are re-included, because host CI reads them: the test
recordings and fixtures (`assets/golden/**`, `assets/*.wav`,
`assets/*.bin`, which start tier A+B alone) and
`mfsk-app-shared/src/**` (which `hosttest/` compiles, and starts the
`ffi` job). Before 2026-09-24 the trigger was `paths-ignore`, which
cannot re-include anything, so replacing a golden recording ran no CI.

`bindings/` is not Rust and so is in neither list — no `Cargo.toml`,
so the workspace never sees either of them:

- `bindings/kotlin/` — the maintained Kotlin/JNI binding (C shim +
  `Mfsk.kt`), built and run on a desktop JVM by the `kotlin` CI job.
  `bindings/kotlin/build.sh` is the local equivalent; it needs
  `JAVA_HOME` and `kotlinc`.
- `bindings/swift/` — a SwiftPM package over the same ABI
  (`import MfskCore`). Its module map includes `mfsk-ffi/include/mfsk.h`
  **in place**, so it follows the header rather than carrying a copy,
  and `bindings/swift/scripts/test.sh` builds `libmfsk` and runs the
  XCTest suite (107 tests as counted by `grep -rc 'func test' bindings/swift/Tests`; all passed on an Apple M5 with Xcode on 2026-10-08, ~10 s — XCTest needs Xcode, not just the
  Command Line Tools, and the script points `DEVELOPER_DIR` at it when
  it has to). The `swift` CI job runs that same script on
  `macos-latest`, and is also where `aarch64-apple-ios` is built: both
  halves need Xcode, one for XCTest and one for the iOS SDK.

A change confined to `bindings/**` runs the two binding jobs and
nothing else — `ci.yml`'s path filter has a `bindings` output beside
`src` for exactly that. Before it existed, `src` did not list
`bindings/**`, so a Kotlin-only PR skipped the Kotlin job.

Everything else:

- `docs/reference/` — the long-form manuals, split by audience since
  2026-09-16: `LIBRARY.md` (Rust host API, trait hierarchy, decode
  strategies), `BINDINGS.md` (the C ABI written from `mfsk.h`, plus
  Kotlin and Swift), `EMBEDDED.md` (fixed-point architecture, FFT
  extern contract, Q-format, per-protocol embedded status),
  `STREAMING.md`, `UVPACKET.md`, and the two operator manuals
  `MANUAL_M5STACK_CORES3.md` / `MANUAL_M5STICKS3.md` / `MANUAL_SKIMMER.md`. **Every file here
  has a `.ja.md` twin** — if you edit one, edit both, or say plainly that
  you didn't.
- `docs/notes/` — `ROADMAP.md` (phase status; the "Quick file-path index"
  at the end is the fastest way to find a subsystem), `BENCHMARKS.md`
  (what a reader is shown: verdict and speed against WSJT-X up top),
  `UPSTREAM_EVALUATION.md` (the evaluation sheet behind that headline:
  paired counts, tests, timings), the per-protocol `*_BENCHMARK.md`, `SNR_FORMULAS.md`, `DECODED_ROW.md`,
  and `sweep-baseline.json` (the machine-checked half of the sensitivity
  story: 50%-crossing SNRs, and for ft8/ft4/fst4 the unexpected-decode
  count per SNR cell).
- `docs/historical/` — split-out older CHANGELOG sections. Grep here as
  well as `CHANGELOG.md` when researching precedent.
- `scripts/` — `build_*sim.sh` / `gen_*_sweep_wavs.sh` build WSJT-X's
  Fortran simulators and generate the tier-C corpora;
  `run-sensitivity-sweeps.sh` + `sweep-regression-check.py` run and diff
  them; `release-status.sh` computes release state; `install-hooks.sh`
  (a wrapper for `git config core.hooksPath .githooks`) and
  `pre-push-check.sh` are the local gates.
- `embedded-poc/assets/` — test audio. `golden/` holds the vendored tier-B
  recordings (with a README mapping each to its WSJT-X upstream);
  `*_sweep/` are the generated tier-C corpora, ~17 GB, gitignored per
  directory.


