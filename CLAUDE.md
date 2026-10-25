# mfsk-core — agent notes

Rules and the map for assistants working in this repo. Normative content is stated here directly; procedures and
background are one link away (see **Where to read next**). Sub-crates have their own `CLAUDE.md` / `README.md`.

## 1. What this is

A pure-Rust 1:1 port of the WSJT-X decoders and synthesisers: FT8, FT4, FST4, FST4W, WSPR, JT9, JT65, Q65 (ten
sub-modes), MSK144, JTTY, and the experimental `uvpacket`. License: GPL-3.0-only.

Around the library: an embedded line (M5Stack CoreS3: USB audio from an IC-705, FT8 / FT4 / JTTY / WSPR, CAT) and a
SpyServer skimmer app.

| path | what |
|---|---|
| `mfsk-core/` | the library (crates.io); everything else consumes it |
| `mfsk-ffi/`, `mfsk-ffi-abi/` | C ABI over all protocols; `include/mfsk.h` is cbindgen-generated and committed |
| `bindings/{kotlin,swift}/` | JNI / SwiftPM over the same ABI; own CI jobs |
| `hosttest/mfsk-app-shared/` | host-testable half of `embedded-poc/mfsk-app-shared` |
| `apps/skimmer/` | SpyServer skimmer (core, cli, Tauri GUI); **own version and own tags** `skimmer-vX.Y.Z`, takes `mfsk-core` by path |
| `embedded-poc/`, `bench/wasm/`, `wasm-smoke/` | outside the workspace: other toolchains / targets |
| `docs/reference/` | manuals; every file has a `.ja.md` twin |
| `docs/notes/` | design rationale, procedures, per-protocol notes, benchmarks |

The library's single version is `workspace.package.version` in the root `Cargo.toml`.

## 2. Principles (the non-negotiables)

1. **WSJT-X is the reference.** Fidelity first; a deliberate divergence is allowed but must be marked in a comment.
   Reference is **v3.2.0-rc1** (plus the results-changing parts of v3.3.0-beta1), tracked on the `release/3.x` line,
   never `master`.
2. **Cite the upstream.** Every algorithm file names the `lib/*.f90` / `lib/*.c` it ports. Changing a decoder
   internal = read the Fortran, make the matching change, keep the `Ported from…` pointer valid. That is part of
   the diff.
3. **Measure, don't assert.** Comments and docs carry evidence (what was measured, on what corpus), not intent.
   Rank bottleneck hypotheses by measurement; name a cause only after measuring it. Measure embedded work on the
   shipped numeric path (`fixed-point`), not f32.
4. **Wide-band is the main path for every mode.** If it is not WSJT-X-faithful without a sniper, that is a bug in
   the wide-band path. Sniper is an FT8-only roofing-filter mode; AP is a general option.
5. **Recall without precision is not a result.** Every decode path is guarded against phantom decodes (§4).
6. **One source, many targets.** Host (rustfft, rayon), `wasm32`, and `no_std` embedded share code behind the
   zero-cost `Protocol` trait. MSK144 and JTTY sit outside it by design (not FSK / no slot) — not a gap to close.
7. **Derive, don't duplicate.** Identities are derived; design-dependent values are required with const helpers;
   split traits so a type only carries what it reads.
8. **Say what you didn't do.** If a check, a twin doc, or a platform was skipped, state it.

## 3. Design map

- The shared module is `src/engine/` — **there is no `src/core/`**; do not invent one to fix a build error.
- **Public decode API is `Decoder<P>`** (`mfsk-core/src/decoder/`): one persistent decoder per mode,
  `DecodeParams` (= `jt9com`) + per-mode `Extras`, `decode(&SlotInput)`; transmit is `encoder::encode(mode, text)`. The
  decode pipeline (`engine::pipeline`) is `pub(crate)`, and `internal-testing` only adds `mfsk_core::__internal` (the tests' and
  embedded's way in, no semver); no other path depends on a feature. The rest is public on purpose, in three tiers (application /
  protocol building / building blocks, `LIBRARY.md` §4) for application builders and for people building their own protocols.
- Adding a protocol: implement the traits (ModulationParams → FrameLayout → Protocol), feature-gate it, fill every
  test cell in §4. Steps: `CONTRIBUTING.md` "Adding a new protocol".
- **Feature traps:** `fixed-point` implies `nstep-half`; `wspr-ddc` and `wspr-ddc-cascade` are mutually exclusive;
  `internal-testing` is required for whole-crate commands (without it clippy shows `E0603`, not your regression).
- **Deliberately not ported** (read the record, don't re-derive): SuperFox (#436, code table licensed for WSJT-X
  only), FT2 (#437), WS's 77-bit extension (#568). Something tempting outside WSJT-X: check
  `docs/notes/UPSTREAM_EVALUATION.md` and these records, then ask. Describe other projects by what a file says,
  never by intent, compliance or fault.
- Why things are the way they are: `docs/notes/DESIGN_RATIONALE.md`; architecture: `docs/notes/CORE_INTERNALS.md`.

## 4. Testing

Tests are organised by the **property asserted**, not location.

| tier | asserts | runs | corpus |
|---|---|---|---|
| **A** invariants | unit, roundtrip, bit-exact parity, TX waveform | every PR | synthetic |
| **B** golden fidelity | recall floor **and** phantom ceiling **and** SNR accuracy vs a reference decoder | every PR | vendored, **missing = failure** |
| **C** sensitivity | AWGN/fading threshold curves + unexpected decodes per SNR cell | before a release / after a sensitivity-moving change | generated, ~17 GB |

- Golden tests go through `assert_golden` (recall + precision together). Target `max_extra: 0`; a nonzero budget
  is documented debt. A test that cannot fail is deleted, not kept.
- **Every decode strategy is guarded**, not just `decode()`: the phantom-prone code is the non-default strategies
  (SIC rounds, `sic_early`). A new strategy ships with its precision guard in the same PR.
- What to run: decoder internals → that protocol's A+B (precision especially); `engine/**` → A+B for every protocol on
  that path; FEC/LLR/decoder algorithm → A+B and tier C before the next release; SNR formula → tier B vs real
  `jt9`/`wsprd`; TX/envelope → A for all protocols. Table: `CONTRIBUTING.md`.
- **Tests must not depend on wall clock or device speed**; use virtual time and reproducible data.
- **Fixture paths:** never hardcode `/home/...` or `/Users/...`. In-repo via `asset_path!` or
  `concat!(env!("CARGO_MANIFEST_DIR"), "/../embedded-poc/assets/<f>")`; out-of-tree via
  `option_env!("WSJTX_SAMPLES_DIR")`, skipping cleanly. (`/tmp` is fine for diagnostic output.)

Commands:

```sh
bash scripts/install-hooks.sh     # core.hooksPath → .githooks (pre-commit AND pre-push)
# merge gate (tiers A+B, ~90 s):
MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core --features full,internal-testing --release
# "is it green?" — pre-push, the gate, the gate again on a clean checkout of HEAD, skimmer, embedded.
# Run it directly (in the background if long); its LAST line is `VERIFY PASS` / `VERIFY FAIL: <stage>`:
scripts/verify.sh
scripts/ci-wait.sh --need 'Kotlin|Swift'   # after a push: CI for HEAD, one last line `CI PASS` / `CI FAIL: …`
```

- `pre-commit`: fmt, clippy `-D warnings` (`--features full,internal-testing`), rustdoc `-D warnings`.
  `pre-push` (`scripts/pre-push-check.sh`): adds `-D clippy::perf`, the feature matrix, `mfsk-ffi` tests, FT8
  fixed-point floors.
- Set `MFSK_REQUIRE_CORPUS=1` locally too, or a mis-resolved asset path silently skips. FST4 tests need `--release`.
- **A test passes only if it passes on a clean checkout.** Generated corpora are `.gitignore`d (`embedded-poc/assets/*_sweep`):
  tier B reads vendored files or a signal synthesised in the test, never those. `scripts/verify.sh` runs the gate on a clean
  worktree of HEAD for this. Read the verdict line, not a notification's exit code (`AGENT_WORKFLOW.md` §8).
- One protocol: `--test <binary> -- --nocapture`. **Never a blanket `-- --ignored`** (that is tier C: hours).
  Tier C: `scripts/run-sensitivity-sweeps.sh [modes]`, only for code paths shared with that mode.
- Host CI never builds `embedded-poc/**`: a green badge on an embedded-only change means nothing compiled it.
- Toolchain is pinned in `rust-toolchain.toml`; raise it only after `toolchain-canary.yml` is green.
- Detail: `docs/notes/TESTING_GUIDE.md`, `DEV_CHECKS.md`, `TIER_C_MANUAL.md`.

## 5. Development process

- `main` is trunk; PRs (or direct pushes — solo repo) land when a unit is complete. `devel` is for revertible
  experiments. Quality is enforced by the hooks, not by branching.
- **Commits:** short imperative subject (`fix: FST4 sync pattern off-by-one`), blank line, a paragraph on the *why*
  with the issue number and upstream SHA / `file:line` if from WSJT-X.
- **CHANGELOG** entry lands with the PR: top section, ≤ 8 lines — what, why, measured effect, where the detail
  lives. Measurement stories go in `docs/notes/`. Older minor series move to `docs/historical/` at a new minor's release PR.
- **Bilingual docs:** edit `docs/reference/*.md` and the `.ja.md` twin together (exact mirror), or say which you skipped.
- Release PRs carry metadata and docs only; behaviour fixes land first, separately.
- **Tracking upstream:** run `scripts/upstream-diff.sh` mid-cycle (~day 7) and when a new tag appears. Classify each
  change A (alters results) / B (real bug fix) / C (refactor); port A and B; record in the tracking Issue
  (#435, #642); bump `DEFAULT_BASE` after the ports merge. Tags may not be ancestors of later tags: compare trees.
- Before starting: check unpushed commits, worktrees, open PRs and the Issue tracker — earlier work may already exist.
- **Model split: one loop, model chosen by task** (`docs/notes/AGENT_WORKFLOW.md`). Default `/model opusplan`:
  Opus plans in plan mode, the same loop implements on Sonnet. Opus main loop for design, unexplained regressions,
  upstream triage; Sonnet for implementing, docs, CI, mechanical ports. Spawn a subagent only for context bloat
  (sweeps, full test logs → `runner`), independent parallel work, or unanchored review. `reviewer` (Opus) only
  when the tests cannot judge the change: gate moved (floor, `max_extra`, fixture), divergence from WSJT-X, new
  strategy, fixed-point, structural `engine/**`, API. A line-for-line upstream port with tier A+B green needs none. To change model, use `/model`, never a `fork`.
- After delegating to parallel agents: `git status`, grep for probe leftovers, commit by explicit path, spot-check claims.

## 6. Releases

- **Never `cargo publish` locally.** A `vX.Y.Z` tag push triggers `release.yml` (waits for CI and tier A+B, publishes).
  Skimmer: separate tag `skimmer-vX.Y.Z`, four version files + `package.json` must agree; 0.x is a pre-release.
- **Step 0: `scripts/release-status.sh`** — computes release state; don't reconstruct it from memory.
- Order: merge release PR → `git checkout main && git pull` → tier-C sweeps the script names → tag → push tag.
- **Cadence: every 2 weeks** (13–14 days); no opportunistic tags. Escape hatch (security, data loss, user asks) is
  the user's call. New protocol = patch; structural / breaking API = minor; unsure → ask.
- Detail: `docs/notes/RELEASE_PROCEDURE.md`.

## 7. Embedded

Boards: `m5stack-cores3-app` (main: UAC host, FT8/FT4/JTTY/WSPR), `m5stack-s3-app`, `m5stack-core2-app` (wav_sim
only). The user flashes the boards: a host check is **not enough** for `src/main.rs` or runtime-path changes —
offer to flash and capture a log. Build with `cargo run --release` (espflash) inside the crate; capture logs only
via `embedded-poc/scripts/flash-monitor.sh` (no second `espflash monitor`, no `cat /dev/ttyACM0`; re-flashing an
unchanged ELF is not a flash). Never run `cargo fmt --all` with the `+esp` toolchain. Read `embedded-poc/CLAUDE.md`
first ("USB host VBUS on CoreS3", "Stacks, heaps, and the space between them").
Procedure: `docs/notes/EMBEDDED_WORKFLOW.md`, `docs/reference/MANUAL_M5STACK_CORES3.md`.

## Where to read next

| need | read |
|---|---|
| contribution rules, porting rules, test philosophy | `CONTRIBUTING.md` |
| Rust host API / C, Kotlin, Swift / no_std + fixed-point | `docs/reference/LIBRARY.md` / `BINDINGS.md` / `EMBEDDED.md` |
| why a design is the way it is | `docs/notes/DESIGN_RATIONALE.md`, `CORE_INTERNALS.md` |
| repo map, CI path filters, what CI doesn't compile | `docs/notes/REPO_MAP.md` |
| features, local checks | `docs/notes/DEV_CHECKS.md` |
| which tests to run, tier C | `docs/notes/TESTING_GUIDE.md`, `TIER_C_MANUAL.md` |
| releasing | `docs/notes/RELEASE_PROCEDURE.md` |
| hardware | `embedded-poc/CLAUDE.md`, board `CLAUDE.md`, `docs/notes/EMBEDDED_WORKFLOW.md` |
| model choice, when to spawn agents, review requirement | `docs/notes/AGENT_WORKFLOW.md` |
| plans, status | `docs/notes/ROADMAP.md`; embedded perf log: auto-memory `project_decode_block_embedded.md` |
