# mfsk-core — agent notes

Repo-level notes for assistants. Sub-crates have their own `CLAUDE.md` / `README.md`. This file is the short
map and the rules that are easy to forget; the long procedures live in `docs/notes/` (linked below, nothing
was dropped when this file was cut down on 2026-10-12).

**Cold start:** this file → `CONTRIBUTING.md` (testing philosophy, WSJT-X porting rules) → whichever of
`docs/reference/LIBRARY.md` (Rust host API), `BINDINGS.md` (C / Kotlin / Swift) or `EMBEDDED.md` (no_std,
fixed-point) the task lands in. Before touching hardware: `embedded-poc/CLAUDE.md` and the board crate's `CLAUDE.md`.

## What this repo is

A pure-Rust 1:1 port of the WSJT-X decoders and synthesisers: FT8, FT4, FST4, FST4W, WSPR, JT9, JT65, Q65 (ten
sub-modes), MSK144, JTTY, and the experimental `uvpacket`. Seven families sit behind a zero-cost `Protocol`
trait (MSK144 and JTTY sit outside it by design), so one source builds for host (rustfft, rayon), `wasm32` and
`no_std` embedded with a fixed-point path. **WSJT-X stays the reference**; every algorithm file cites the
`lib/*.f90` / `lib/*.c` it ports, and keeping those citations valid is part of the diff. License: GPL-3.0-only.

Alongside the library: an embedded line (M5Stack CoreS3: USB audio from an IC-705, FT8 / FT4 / JTTY / WSPR, CAT
over the same cable) and a SpyServer skimmer app.

## Repository map

| path | what | published |
|---|---|---|
| `mfsk-core/` | the library; everything else consumes it | crates.io |
| `mfsk-ffi/`, `mfsk-ffi-abi/` | C ABI over all protocols (`include/mfsk.h` is cbindgen-generated and committed); shared `#[repr(C)]` types | no |
| `hosttest/mfsk-app-shared/` | runs the host-testable half of `embedded-poc/mfsk-app-shared` under `cargo test` | no |
| `apps/skimmer/{core,cli}/` | the SpyServer skimmer; **its own version** (1.0.0 on 2026-10-12, PSK Reporter in production) and its own tags `skimmer-vX.Y.Z` | GitHub release only |
| `apps/skimmer/gui/` | Tauri + Svelte GUI over `skimmer-core`; outside the workspace | with the skimmer release |
| `bindings/{kotlin,swift}/` | Kotlin/JNI and SwiftPM bindings over the same ABI; not Rust, own CI jobs | no |
| `embedded-poc/`, `bench/wasm/`, `wasm-smoke/` | outside the workspace (`exclude`): other toolchains / targets | no |
| `docs/reference/` | long-form manuals; **every file has a `.ja.md` twin: edit both, or say which you skipped** | |
| `docs/notes/` | ROADMAP, BENCHMARKS, UPSTREAM_EVALUATION, the per-protocol notes, and the moved sections below | |

`workspace.package.version` in the root `Cargo.toml` is the library's single version (the skimmer has its own, in
four files + `package.json`). The skimmer takes `mfsk-core` **by path**, so a skimmer tag carries the library's
`main` at that commit, not its latest crates.io release. Full map, CI path filters and what CI does *not*
compile: `docs/notes/REPO_MAP.md`.

## Rules that bite

- **There is no `src/core/`; the shared module is `src/engine/`.** Don't "fix" a build error by inventing a
  `core` module (Rust's own `core::sync::atomic` is a different thing).
- **Public decode API is `Decoder<P>`** (`mfsk-core/src/decoder/`, since 0.13.0): one persistent decoder per mode,
  `DecodeParams` (= `jt9com`) plus per-mode `Extras`, `decode(&SlotInput)`. The raw engine functions are
  `pub(crate)` on purpose.
- **Sniper is an FT8-only roofing-filter mode; AP is a general option.** Wide-band is the main path for every
  mode; if it isn't WSJT-X-faithful without a sniper, that is a bug in the wide-band path.
- **MSK144 and JTTY are outside the `Protocol` trait** (not FSK / no slot). Not a gap to close.
- **Phantom-prone code is the non-default strategies** (SIC rounds, `sic_early`): a new strategy ships with its
  precision guard in the same PR.
- **Cite the upstream.** Changing a decoder internal means reading the Fortran first, making the matching
  change, and keeping the `Ported from…` pointer valid. A deliberate divergence gets a comment saying so.
- **Comments carry evidence, not intent:** a knob's doc says what was measured and on what corpus.
- **Feature traps:** `fixed-point` implies `nstep-half` (must); `wspr-ddc` and `wspr-ddc-cascade` are mutually
  exclusive; **`internal-testing` is required for whole-crate commands** (without it clippy shows `E0603`, not
  your regression). Full list: `docs/notes/DEV_CHECKS.md`; the architecture: `docs/notes/CORE_INTERNALS.md`.
- **Test fixture paths:** never hardcode `/home/...` or `/Users/...`. In-repo assets via `asset_path!` or
  `concat!(env!("CARGO_MANIFEST_DIR"), "/../embedded-poc/assets/<f>")`; out-of-tree via
  `option_env!("WSJTX_SAMPLES_DIR")`, skipping cleanly. (`/tmp/...` is fine for diagnostic output.)

## Local checks and tests

```sh
bash scripts/install-hooks.sh            # = git config core.hooksPath .githooks (pre-commit AND pre-push)
```

- `pre-commit`: `cargo fmt --check`, clippy `-D warnings` (`--features full,internal-testing`), rustdoc `-D warnings`.
- `pre-push` (`scripts/pre-push-check.sh`): adds `-D clippy::perf`, the **feature matrix** (read `FEATURE_MATRIX` in
  the script), `mfsk-ffi` tests, the FT8 fixed-point floors.
- **Merge gate (tiers A + B), ~90 s:**

  ```sh
  MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core --features full,internal-testing --release
  ```

  Set `MFSK_REQUIRE_CORPUS=1` locally too, or a mis-resolved asset path silently skips. FST4 tests must use `--release`.
- **One protocol:** scope with `--test <binary> -- --nocapture`. **Never a blanket `-- --ignored`**: that
  escalates into tier C (hours, ~17 GB corpora). Tier C runs only before a release tag or after a change that
  moves sensitivity: `scripts/run-sensitivity-sweeps.sh [modes]`, and only if the code path you touched is shared
  with that mode. Details, `fixed-point` and `wspr-ddc` runs: `docs/notes/TESTING_GUIDE.md`.
- Host CI never builds `embedded-poc/**`: **a green badge on an embedded-only change means nothing compiled it.**
- The host toolchain is pinned in `rust-toolchain.toml` (CI checks it); raise it only after `toolchain-canary.yml` is green.

## Embedded

Boards: `m5stack-cores3-app` (main target: UAC host, FT8/FT4/JTTY/WSPR), `m5stack-s3-app`, `m5stack-core2-app`
(wav_sim only). The user flashes them: **don't assume a host check is enough** for `src/main.rs` or runtime-path
changes; offer to flash and capture a log. Build with `cargo run --release` (espflash) inside the crate, and
capture logs only through `embedded-poc/scripts/flash-monitor.sh` (never a second `espflash monitor`, never
`cat /dev/ttyACM0`; a re-flash of an unchanged ELF is not a flash). Read `embedded-poc/CLAUDE.md` first
("USB host VBUS on CoreS3", "Stacks, heaps, and the space between them"). Measure on the shipped numeric path
(`fixed-point`). Procedure and status: `docs/notes/EMBEDDED_WORKFLOW.md`, `docs/reference/MANUAL_M5STACK_CORES3.md`.

## Releases

- **Never `cargo publish` locally.** A `vX.Y.Z` tag push triggers `release.yml`, which waits for CI (and for the
  tier A+B job specifically) and publishes. Skimmer releases are a separate tag, `skimmer-vX.Y.Z`
  (`skimmer-release.yml`; four version files must agree; 0.x is a pre-release).
- **Step 0: `scripts/release-status.sh`.** It computes the release state (tagged? CHANGELOG current? cadence? which
  protocols' sweeps are outstanding). Don't reconstruct it from memory or from this file.
- Order: merge the release PR → `git checkout main && git pull` → tier-C sweeps the script names → tag → push the tag.
- **Cadence: every 2 weeks** (13-14 days is the target) for the library; don't tag opportunistically. Escape hatch
  (security, data loss, or the user asks) is the user's call. New protocol = patch bump by convention; structural / breaking API = minor; unsure → ask.
- Release PRs carry metadata and docs only; behaviour fixes land first, separately.
- Full procedure, sweep baseline, cadence history: `docs/notes/RELEASE_PROCEDURE.md`.

## Following WSJT-X

Live upstream is `WSJTX/wsjtx` (the `upstream` remote of `../WSJT-X`), development line `release/3.x` and its
`v3.x.0-*` tags, not `master`. Reference is **v3.2.0-rc1**, with results-changing parts of **v3.3.0-beta1** ported
(#642). Run `scripts/upstream-diff.sh` at the midpoint of the release cycle (about day 7) and when a new tag
appears; classify each change A (alters results) / B (real bug fix) / C (refactor); port A and B; record in an
Issue (#435, #642); bump `DEFAULT_BASE` after the ports merge. A tag is not necessarily an ancestor of a later
tag: compare trees. Last run 2026-10-12: nothing new past beta1.

## Scope: deliberately not ported (read the record, don't re-derive)

| left out | why | record |
|---|---|---|
| **SuperFox** | its QPC (127,50) code table is licensed for WSJT-X only; this crate is GPL-3.0-only | #436 |
| **FT2** | not part of WSJT-X; no specification of record; `ProtocolId::Ft2 = 2` stays reserved | #437, `docs/notes/FT2_INVESTIGATION.md` |
| **WS's 77-bit extension** | bit layouts are WSJT-X's; only the QSO text differs; received like any other, no WS code taken | #568, `docs/notes/WS_77BIT_EXTENSION.md` |

Write about other projects by what a file says and what a note found; leave their intent, compliance and fault
out, in chat, commits, issues and docs alike. Not legal advice. Something tempting outside WSJT-X: check this
table, then ask.

## Branching and conventions

- `main` is trunk; PRs (or direct pushes, solo repo) land when a unit is complete. `devel` is for open-ended
  experiments that may be reverted.
- **Commit messages:** short imperative subject (`fix: FST4 sync pattern off-by-one`), blank line, a paragraph on
  the *why* with the issue number and the upstream SHA / `file:line` if it came from WSJT-X.
- **CHANGELOG entries land with the PR**, top section, eight lines or fewer: what, why, the measured effect, where
  the detail lives. Measurement stories go in `docs/notes/`. `CHANGELOG.md` holds the unreleased section and the
  two latest minor series; older ones move to `docs/historical/` at each new minor's release PR.
- **Bilingual docs:** keep `docs/reference/*.md` and their `.ja.md` twins in step.
- **Don't hardcode machine paths** anywhere a test can reach.

## Memory

`~/.claude/projects/-home-minoru-src-mfsk-core/memory/` holds the per-conversation auto-memory.
`project_decode_block_embedded.md` is the authoritative log of the embedded performance work; read it before
touching `decode_block` or the production app crates.
