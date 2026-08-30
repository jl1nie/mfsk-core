# mfsk-core — agent notes

Repo-level notes for assistants working in this tree. Anything specific to
a sub-crate lives in its own `CLAUDE.md` or `README.md`; this file is for
the cross-crate map and workflow that's easy to forget between sessions.

Read order for a cold start: this file, then `CONTRIBUTING.md` (testing
philosophy, WSJT-X porting rules), then whichever of
`docs/reference/LIBRARY.md` (host API, ~1700 lines) or
`docs/reference/EMBEDDED.md` (no_std / fixed-point / FFI-to-C) the task
lands in. `embedded-poc/CLAUDE.md` plus the per-board crate `CLAUDE.md`
before touching hardware.

## What this repo is

A pure-Rust 1:1 port of the WSJT-X decoders and synthesisers — FT8, FT4,
FST4, WSPR, JT9, JT65, Q65 (30A plus 60A‥E), MSK144, and the experimental
`uvpacket` mode — behind a zero-cost `Protocol` trait, so the same source
builds for a host (rustfft, rayon), for `wasm32`, and for `no_std`
embedded targets with a fixed-point hot path. WSJT-X stays the reference
implementation; this crate exists to put those decoders on platforms
WSJT-X can't reach. Every algorithm file cites the `lib/*.f90` / `lib/*.c`
it ports, and keeping those citations valid is part of the diff.

Alongside the library, the repo carries a working embedded line: an
M5Stack CoreS3 that takes USB-audio from an IC-705 and decodes FT8, FT4,
FST4 and WSPR off the air.

## Repository map

Host workspace members (`Cargo.toml` `[workspace] members`):

| path | what it is | published? |
|---|---|---|
| `mfsk-core/` | the library — everything below is a consumer of it | **yes**, crates.io |
| `mfsk-ffi/` | C ABI over *all* protocols; `include/mfsk.h` is cbindgen-generated and committed; `examples/cpp_smoke/` is a real C++ driver run by CI | no |
| `mfsk-ffi-ft8/` | embedded-friendly C ABI over the FT8 slice only; host **and** `embedded-fixed-point` (pure `no_std`) builds; ships as prebuilt tarballs on GitHub Releases | no |
| `mfsk-ffi-abi/` | shared `#[repr(C)]` status/result/options types both FFI crates re-emit (issue #205) | no |
| `hosttest/mfsk-app-shared/` | runs the host-testable parts of `embedded-poc/mfsk-app-shared` under a normal `cargo test` | no |

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

`ci.yml` `paths-ignore`s both trees, so a change confined to them runs no
host CI at all. That is intentional, and it is also why "CI is green" says
nothing about an embedded change.

Everything else:

- `docs/reference/` — the long-form manuals: `LIBRARY.md` (host API,
  trait hierarchy, decode strategies, FFI), `EMBEDDED.md` (fixed-point
  architecture, FFT extern contract, Q-format, per-protocol embedded
  status), `STREAMING.md`, `UVPACKET.md`, and the two operator manuals
  `MANUAL_M5STACK_CORES3.md` / `MANUAL_M5STICKS3.md`. **Every file here
  has a `.ja.md` twin** — if you edit one, edit both, or say plainly that
  you didn't.
- `docs/notes/` — `ROADMAP.md` (phase status; the "Quick file-path index"
  at the end is the fastest way to find a subsystem), `BENCHMARKS.md`,
  the per-protocol `*_BENCHMARK.md`, `SNR_FORMULAS.md`, `DECODED_ROW.md`,
  and `sweep-baseline.json` (the machine-checked half of the sensitivity
  story).
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

## `mfsk-core` internals — the map before you edit

A protocol is a **zero-sized type** implementing `ModulationParams`
(tones, symbol rate, Gray map, GFSK shaping), `FrameLayout` (symbol
counts, slot length, sync blocks), and `Protocol` (which binds
`type Fec` and `type Msg`). Everything generic is monomorphised per `P`,
so the abstraction costs compile time and nothing else. Adding a protocol
is a trait impl plus a registry line — FST4-60A landed without touching
shared code. `CONTRIBUTING.md` "Adding a new protocol" and
`LIBRARY.md` §2 are the step-by-steps.

| module | contents |
|---|---|
| `engine/` | the shared core: `protocol.rs` (the traits), `pipeline.rs`, `sync.rs` / `sync2d.rs`, `spectrogram.rs`, `llr.rs`, `equalize.rs`, `scalar.rs` (Q-format types), `fft.rs` (the `FftPlanner` trait + extern factory), `tx.rs`, and `dsp/` (resample, downsample, GFSK, envelope, subtract, DDC, FIR/polyphase, dotprod, the fixed-point FFT kernels) |
| `fec/` | `ldpc/` (174,91), `ldpc240_101/`, `ldpc_128_90/` (MSK144), `conv/` (r=½ K=32 Fano), `rs/` (63,12 over GF(2⁶)), `qra/` + `qra15_65_64/` (Q65) |
| `msg/` | message codecs — `wsjt77.rs`, `jt72.rs`, `wspr.rs`, `q65.rs`, `packet_bytes.rs`, `callsign28.rs`, `hash_table.rs` — plus `decode_request.rs` and `decoded.rs`, which are the public entry point and the public output row |
| `ft8/ ft4/ fst4/ wspr/ jt9/ jt65/ q65/ msk144/ uvpacket/` | per-protocol ZSTs, decoders, synthesisers; each feature-gated by its own name |
| `registry.rs` | `PROTOCOLS: &[ProtocolMeta]` + `by_id` / `by_name` / `for_protocol_id` — how a UI or FFI layer asks "what does this build support?" without hardcoding a list |

**There is no `src/core/`. The shared module is `src/engine/`.**
`CONTRIBUTING.md`'s layout block and its `crate::core::pipeline::…`
snippet, and several `Cargo.toml` feature comments (`mfsk-core::core::fft`,
`core::pipeline`, `core::scalar`), still use the old name; the source has
zero `crate::core::` paths. Read those as `engine::`. Don't "fix" a build
error by inventing a `core` module.

**`DecodeRequest` / `SniperRequest` (`msg::decode_request`) are the public
decode API.** Builder-shaped: `.freq_hint()`, `.osd()`, `.strictness()`,
`.eq_mode()`, `.known()`, `.fft_cache()`, `.on_result()`, then `.decode()`.
The raw engine functions underneath (`decode_frame`,
`process_candidate_basic`, the `GenericPipelineProtocol` trait) are
`pub(crate)` on purpose since #191 so downstream can't bypass the request
types — `internal-testing` is what reopens them for integration tests, and
it is deliberately not in `full`.

Strategy extensions are trait-gated per protocol:
`.sic_rounds(n)` needs `SupportsSicRounds`, `.sic_early()` needs
`SupportsSicEarly` — implemented for FT8/FT4 only, mirroring an upstream
absence. **The phantom-prone code lives in these non-default strategies**
(both false-decode bugs this suite has shipped were in subtraction paths:
#243 in `__staged_sic`, #253 in `.sic_early()`), so a new strategy ships
with its precision guard in the same PR.

**MSK144 is intentionally outside the `Protocol` trait** — it isn't FSK,
and `msk144::decode::decode_slot` bypasses `engine::pipeline` by design.
It therefore doesn't appear in `PROTOCOLS` or `protocol_invariants.rs`.
That is an architectural decision, not a gap to close.

`tests/protocol_invariants.rs` runs one generic
`assert_protocol_invariants::<P>` over every wired ZST (11 with `full`;
uvpacket adds four more) and pins ~25 trait-level invariants. A new
protocol gets one line there.

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
- **`jt9` / `jt65` / `q65` are host-only** — they call `rustfft` directly,
  so they pull `fft-rustfft` and therefore `std`. **`fst4` is not**,
  despite living in that block historically: issue #306 confirmed it
  routes entirely through the backend-agnostic `engine` machinery and
  type-checks clean under `alloc,fst4,fft-extern`.
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
  **feature matrix**: `--no-default-features` alone, then each of `ft8`,
  `ft4`, `fst4`, `wspr`, `jt9`, `jt65`, `q65`, `uvpacket`, `alloc ft8`,
  `alloc ft8 fft-extern`, `alloc ft8 fft-extern fixed-point`, `full`.
  This exists because PR #233 shipped code that compiled only under
  `full` — `dead_code` reachability depends on which cfg'd items a given
  feature set exposes, so passing one or two combos proves nothing about
  the rest.

CI (`ci.yml`) then runs: `changes` (a path filter that decides which sweep
suites are relevant), `lint`, `test` (the tier matrix — see below),
`feature-matrix`, `ffi` (`mfsk-ffi` + `mfsk-ffi-ft8` Rust tests, the C++
driver including its multi-thread stress, and `mfsk-app-shared-hosttest`),
`docs`, and `publish-dry-run`. `RUSTFLAGS: -D warnings` is set globally.

## Running tests — pick the tier, never blanket `--ignored`

`CONTRIBUTING.md` "Testing philosophy" defines the four tiers (A
invariants / B golden fidelity / C sensitivity / D deleted). This
section is the operational half: which command to actually type.

**The merge gate — use this by default.** Every non-ignored test
across every protocol, i.e. tiers A + B. ~90 s on a Ryzen 9 9900X.
This is exactly what CI's `Test (tier A+B — invariants + golden)` job
runs:

```sh
MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core --features full,internal-testing --release
```

`MFSK_REQUIRE_CORPUS=1` turns a missing golden recording into a
failure instead of a silent skip. Set it locally too — without it a
mis-resolved asset path reports green (that is exactly how five
protocols' golden tests skipped unnoticed before the recordings were
vendored).

Tier B goes through `tests/common/golden.rs::assert_golden`, which takes
recall **and** precision together. There is deliberately no recall-only
entry point: adding a golden test forces a decision about the protocol's
phantom budget. Target `max_extra: 0`; anything else is documented debt.

**Iterating on one protocol**: scope with `--test`, don't reach for
`--ignored`.

```sh
cargo test -p mfsk-core --features full,internal-testing --release --test ft8_qso3_full_parity_recall -- --nocapture
```

**The trap: a blanket `-- --ignored` locally escalates into tier C.**

```sh
# DON'T — this is a sensitivity measurement campaign, not a check.
cargo test -p mfsk-core --features full,internal-testing --release -- --ignored
```

`--ignored` with no `--test` scope runs *every* `#[ignore]`d function
in the crate, which includes the tier-C sensitivity sweeps
(`fst4_sweep`, `ft8_sweep`, `jt9_sweep`, `jt65_sweep`, `wspr_sweep`,
`q65_*_sweep`, …). Those are the ones that consume the ~17 GB
generated corpora under `embedded-poc/assets/*_sweep/`. **On CI they
silently skip** — the corpora aren't there, and most of those
binaries aren't in any CI glob anyway — so the command looks harmless
in `ci.yml` and is not harmless here. `fst4_sweep` **alone** had not
finished after 35 minutes when it was killed (2026-08-12), and it is
one of a dozen such binaries.

CI does run some `--ignored` suites, but always scoped per binary
(`--test q65_ap_sweep --test q65_snr_sweep … -- --ignored`) and
push-only (or on a PR carrying the `run-full-sweep` label). If you want a
specific ignored test, name its binary the same way.

**Tier C** belongs to two moments only: before a release tag, and
after a change that plausibly moves sensitivity. It has its own
runner, which checks the corpora are present up front instead of
producing a wall of silent skips:

```sh
scripts/run-sensitivity-sweeps.sh              # everything present
scripts/run-sensitivity-sweeps.sh ft4 fst4     # or just what moved
```

**"Plausibly moves sensitivity" means the code path, not the
protocol name.** Before running a protocol's sweep, check it actually
shares the code you touched. FT8's `decode_block::coarse_sync` /
`SYNC_LAG_S` is FT8-only — FST4 reaches sync through
`engine::sync::coarse_sync` + `engine::sync2d::fst4_sync_search`, so
an FT8 coarse-sync change cannot move an FST4 curve and running that
sweep buys nothing (issue #280, where this was learned the expensive
way). `release-status.sh` prints this warning itself whenever a commit
touches `src/engine` — heed it rather than sweeping everything.

**Other feature sets.** `fixed-point` is the numeric path embedded
ships and is worth a second run whenever you touch the decode
pipeline — it reorders candidates through `SpecCell = u16`
quantisation, so it can diverge from host f32 on the same fixture:

```sh
cargo test -p mfsk-core --features full,fixed-point --release --test ft8_qso3_apoff_recall
```

`wspr-ddc` and `wspr-ddc-cascade` are the WSPR embedded channelizer
swaps (`wspr::ddc`'s single-stage and two-stage-cascade streaming
down-converters, replacing the host-default whole-slot FFT
`decimate_to_baseband`) — mutually exclusive (`compile_error!` in
`decode_scan_inner` if both are on), each with its own real-signal
golden test against the WSJT-X recording, worth a run whenever
`wspr::ddc` changes:

```sh
MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core --features full,internal-testing,wspr-ddc-cascade --release --test wspr_wsjtx_samples wspr_cascade_ddc_golden_recall_and_precision
```

**`internal-testing` is not optional for whole-crate commands** — see the
feature-flag section above for why the `E0603`s are not your fault.

## Test fixture paths

Never hardcode absolute paths like `/home/ubuntu/...` or `/Users/...`
for test inputs. AI assistants tend to "fix" path failures by
translating to whichever local environment they happen to run in
(commit `119657a` flipped `/home/minoru/` → `/home/ubuntu/`), which
just relocates the bug.

- **In-repo assets**: use the `asset_path!` macro from
  `mfsk-core/tests/common/mod.rs` (integration tests) or
  `concat!(env!("CARGO_MANIFEST_DIR"), "/../embedded-poc/assets/<f>")`
  (unit tests under `src/`). Vendor the file under
  `embedded-poc/assets/` if it's not already there — the FT8 / JT9
  reference recordings already live there, and the tier-B goldens are
  under `embedded-poc/assets/golden/` (see its README for the upstream
  mapping), resolved through `tests/common/corpus.rs`.
- **Out-of-tree user-machine assets** (e.g. the full WSJT-X tarball):
  `option_env!("WSJTX_SAMPLES_DIR")` and skip cleanly when unset.
- **Diagnostic output paths** (test writes a WAV for human inspection):
  `/tmp/...` literals are fine — the human-in-the-loop step assumes a
  known location. Don't replace these with `tempfile`.

## Embedded targets

The active production crates are `embedded-poc/m5stack-s3-app/` (S3
LX7, repositioned as **demo / acoustic-fallback** in the 2026-05-17
pivot — the StickS3 board can't do USB host),
`embedded-poc/m5stack-core2-app/` (Core2 LX6, wav_sim only — no USB
peripheral on classic ESP32), and `embedded-poc/m5stack-cores3-app/`
(S3 LX7, **main UAC controller target** — CoreS3 has AXP2101 +
AW9523B for proper USB-OTG host mode). Phase 0-Core (board bring-up)
and Phase 1-Core (UAC host code) both shipped 2026-05-23, and
Phase 1-Verify — live IC-705 hardware confirmation, issue #163 —
**cleared 2026-08-23**: ten unbroken minutes of UAC capture, 125 MB,
zero errors, with WiFi associated throughout (the log is kept at
`embedded-poc/m5stack-cores3-app/logs/uac_stream_2026-08-23.log`).
Getting there took three layers off the enumeration path and then two
stack overflows that presented as heap corruption — read
`embedded-poc/CLAUDE.md`'s "USB host VBUS on CoreS3" and "Stacks,
heaps, and the space between them" before touching that board.

**It decodes off the air now (2026-08-23/24)**: FT8 at six to eight
stations per slot against an IC-705 on 40 m, WSPR at
`slot 1 src=uac decoded 1 station(s)`. The board carries all four
receivers in one image, chosen from the touch panel. Operating notes
are in `docs/reference/MANUAL_M5STACK_CORES3.md` (and `.ja.md`); the
crate has its own `CLAUDE.md` for what breaks a session.
See `docs/notes/ROADMAP.md` Phase B-Core for what #163 unblocks. `m5stack-s3-app` and
`m5stack-core2-app` each have their own `CLAUDE.md` covering
board-specific bring-up, and `m5stack-cores3-app` now does too —
alongside the operator manual at
`docs/reference/MANUAL_M5STACK_CORES3.md` / `.ja.md`. This
section captures the shared workflow that's easy to forget between
sessions.

Shared code for these boards lives in `embedded-poc/embedded-shared/`
(decode-side glue, esp-dsp FFT/dotprod backends) and
`embedded-poc/mfsk-app-shared/` (UI/app logic, whose host-testable half
runs in the workspace as `hosttest/mfsk-app-shared`).
`embedded-poc/idf-component/` is the esp-idf bridge template for C
projects consuming `mfsk-ffi-ft8`.

**WSPR embedded RX (Phase E, issue #260) is a separate track from the
FT8-controller line above** — it never goes through `decode_block` or
the UAC/controller stack, so it isn't blocked on #163 the way Phase
B-Core is (both shared #163 only for live audio capture; everything
else already works against WAV-fed/synthetic baseband, and #163 itself
cleared 2026-08-23 on the FT8 controller). Lives in the
same `m5stack-cores3-app` crate as two separate binaries:
`src/bin/wspr_bench.rs` (timing measurement) and `src/bin/wspr_app.rs`
(the standalone receiver — LCD spot list, WiFi, HTTP config, NTP, and
real UAC audio through `AudioSink`). Note that `wspr_app` *does* now
share `uac.rs` with the FT8 line; that path is proven on the FT8
controller but `wspr_app` itself has not been run against a radio yet
— open items in #313. See `docs/reference/EMBEDDED.md`'s "WSPR on
embedded" section and `docs/notes/ROADMAP.md` Phase E for status.

- **Build & flash via `espflash`**, not host cargo. Each crate's
  `.cargo/config.toml` sets `runner = "espflash flash --monitor"`, so the
  basic user workflow is:
  ```sh
  cd embedded-poc/m5stack-cores3-app   # or m5stack-s3-app, m5stack-core2-app
  cargo run --release                  # builds + flashes + opens serial monitor
  ```
  In practice, for capturing per-session logs to a file, use
  `embedded-poc/scripts/flash-monitor.sh` (see next section) instead of
  the bare runner.
- The `+esp` Rust toolchain (Xtensa fork, espup-installed) is selected
  by each crate's `rust-toolchain.toml`. Target triple per
  `.cargo/config.toml`: `xtensa-esp32-espidf` for Core2 (LX6),
  `xtensa-esp32s3-espidf` for S3 / CoreS3 (LX7).
- `cargo check` from inside each crate validates code changes without
  flashing (~30-50 s with prebuilt esp-idf).
- Host CI never builds these crates (`ci.yml` `paths-ignore`s
  `embedded-poc/**`). A green CI badge on an embedded-only change means
  the change was not compiled by anything.
- Logs from the device land in `embedded-poc/<crate>/logs/` — user has
  been capturing per-session sweep output there.
- The user actively flashes these boards during embedded work; do NOT
  assume "host check is enough" when changing `src/main.rs` or anything
  that affects the runtime path. Offer to flash and capture a new log.
- The compute-bench crate `embedded-poc/m5stack-s3/` still exists for
  S3-only decoder timing sweeps. The Core2 bench (`m5stack-core2/`) was
  retired in `#61` Phase 3 — `m5stack-core2-app` covers the same
  wav_sim decode path in a production-app shape.

## Capturing logs from a flashed device (ESP32 / S3)

Use `embedded-poc/scripts/flash-monitor.sh` — **never** roll your own
`espflash flash --monitor` + redirect, and never `cat /dev/ttyACM0`. Two
foot-guns this script avoids:

1. A **separate** `espflash monitor` invocation resets on connect, and
   on S3 USB-OTG boards that drops the chip into DOWNLOAD mode
   (`rst:0x15 USB_UART_CHIP_RESET … waiting for download`) so the
   just-flashed app never runs. The script avoids it by never starting
   a second process: one `espflash flash --monitor` covers both.

   It does **not** pass `--before no-reset --after no-reset`, as this
   note used to claim — those would break flashing outright.
   `espflash flash`'s own defaults are already what is wanted:
   `--before default-reset` is how it gets into the bootloader to write
   at all, and `--after hard-reset` is what starts the new image.

   Boards have still been found parked in DOWNLOAD mode twice
   (2026-08-23), both times *after* the capture window expired and
   `timeout` killed the pty rather than after the write itself. The
   port closing is the suspect, not the flash. Unconfirmed. A short
   press of the board's button boots the app; nothing is lost, since
   the image is already in flash and NVS is a different partition.

2. Re-flashing the same ELF prints "Segment … has not changed, skipping
   write" and finishes in ~5 s. **That is not a successful flash** — the
   chip still runs the previous binary. Touch a source file or change a
   `log::info!` line to force a real rewrite, and expect ~15-25 s for a
   real factory-partition write.

```sh
source ~/export-esp.sh
cd embedded-poc/m5stack-s3-app   # or m5stack-s3, m5stack-core2-app, m5stack-cores3-app
cargo build --release --bin <bin>
../scripts/flash-monitor.sh \
    target/<triple>/release/<bin> \
    logs/<bin>_<tag>_$(date +%Y-%m-%d).log \
    90    # capture seconds (optional, default 90; use ≥120 for fresh Core2 flashes — 1.3 MB binary takes ~55 s to write)
```

## Releases — go through CD, never `cargo publish` locally

`mfsk-core` ships to crates.io via `.github/workflows/release.yml`,
triggered by a `vX.Y.Z` tag push. The workflow gates the publish on
the CI for the same commit going green (`wait-for-ci` job), then
runs `cargo publish -p mfsk-core --features full` + builds the
`mfsk-ffi-ft8` FFI artifacts + creates the GitHub release with
attached tarballs.

`wait-for-ci` checks two things, not one. The workflow-run poll
tolerates `skipped` (a docs-only push legitimately skips the build
matrix), which is too wide for the job carrying the golden tier-B
assertions — so a second step requires `Test (default)` to have
concluded `success` at *job* granularity. Together with
`MFSK_REQUIRE_CORPUS=1` in `ci.yml`, a green `Test (default)` is a
positive statement that the golden tests ran against real recordings
rather than skipping. Before the recordings were vendored they did
skip, silently, for five protocols.

**Do not `cargo publish` from a local clone.** crates.io publishes
are irreversible — once a version is up, you cannot unpublish
(yank exists but blocks new dependents while existing dependents
keep using the broken version). A local publish bypasses the CI
gate that this whole workflow exists to enforce; even if your
local SHA happens to be CI-green, future releases get sloppier
when "just publish locally" is in the playbook. Push the tag and
let CD do it.

**Step 0, before anything else: `scripts/release-status.sh`.**

It prints the release state computed from the repository — whether the
workspace version is tagged, whether the CHANGELOG's top section agrees
with it, how the cadence stands, and which protocols' own source has
changed since `sweep-baseline.json` was last refreshed, with the commit
subjects so "clippy drift" and "decoder change" are distinguishable at a
glance.

This exists because release preparation kept failing the same way: the
state was reconstructed by hand, from memory and inference, and every
fact involved is mechanically derivable. Three errors in one session on
2026-08-23 — claiming sweeps were outstanding when a note recorded them
as done with an explicit re-run condition that was never evaluated;
naming three protocols as needing re-sweeping on the strength of commits
that predated the sweep; and not noticing that `0.10.0` had its
CHANGELOG written and its version bumped but no tag, through an entire
conversation about preparing a release. The procedure below was already
written down. What was missing was anything that computed the state, so
each attempt re-derived it, and re-derivation is where the errors came
from.

For the same reason, don't take a release-state claim from this file
either: as of 2026-08-24 the script reports `0.10.0` bumped, its
CHANGELOG section written and current, and **no tag** — but run it rather
than quoting that sentence.

Sequence:
1. Merge release PR into `main`.
2. `git checkout main && git pull`.
3. **Run the tier-C sensitivity sweeps** (see below).
4. `git tag vX.Y.Z <merge-sha>` then `git push origin vX.Y.Z`.
5. Watch the Actions tab for the `Release` workflow.

### Step 3: tier-C sensitivity sweeps, before every tag

**Which ones, from `scripts/release-status.sh` — not from recall.** It
compares each protocol's own source tree against the date
`sweep-baseline.json` was last refreshed and prints the commit subjects,
so a clippy sweep and a decoder change are told apart without going to
look them up.

```sh
scripts/release-status.sh                      # says which, and why
scripts/run-sensitivity-sweeps.sh fst4         # then just those
```

CI never runs these. They need the generated corpora under
`embedded-poc/assets/*_sweep/` (~17 GB, gitignored, built by
`scripts/gen_*_sweep_wavs.sh` on top of WSJT-X's Fortran simulators),
so on CI every one of them silently skips. A release is the interval
where that gap actually matters — shipping a sensitivity regression to
crates.io is the outcome the sweeps exist to prevent, and it is
irreversible once published.

They **assert nothing** by design: sensitivity is a curve, and a
threshold that moved 0.3 dB is a judgement call rather than a boolean.
The script now diffs itself: every recall-vs-SNR sweep it runs dumps a
per-trial CSV to `target/sweep-csv/`, and at the end it runs
`scripts/sweep-regression-check.py` against those CSVs, which
interpolates each channel's 50%-crossing SNR and prints the delta from
`docs/notes/sweep-baseline.json`, flagging `!!` on any move
`>= 0.5 dB`. This replaced manually eyeballing the printed tables (or
spawning several agents to do it) against `docs/notes/*BENCHMARK.md` —
see `~/.claude/projects/.../memory/project_sensitivity_sweep_pre_release_20260814.md`
for what that used to cost. Still sanity-check the prose in
`docs/notes/*BENCHMARK.md` too — the JSON baseline only tracks
50%-crossing SNR, not e.g. WSPR's phantom-decode count. Once you
understand why a number moved, refresh the baseline with
`python3 scripts/sweep-regression-check.py --update-baseline
target/sweep-csv/*.csv`, and update `docs/notes/BENCHMARKS.md` too if
the reason is one worth recording there (new hardware counts — the
table records the machine).

`--update-baseline` also stamps the JSON's `_meta.protocols` block —
date, commit, machine, trial count — **for the protocols in that run
only**, which is what `release-status.sh` reads to decide what is
actually outstanding. So a partial refresh
(`run-sensitivity-sweeps.sh ft4 fst4`) silences ft4 and fst4 and
leaves the other seven still flagged, instead of the whole file
looking current because one date moved. Don't hand-edit those dates:
the point is that the number and its provenance are written by the
same run.

A nightly workflow was considered and rejected: on a solo, bursty repo
most nights would re-measure unchanged code, and this project already
deleted one scheduled tier for precisely that reason (see `ci.yml`'s
note that it "only ever cost wall-clock for output nobody was
routinely reading").

### Release cadence — biweekly, decoupled from merging

PRs land on `main` immediately as they're ready — CHANGELOG.md's
top (unreleased/latest-numbered) section accumulates entries between
tags, so `main`'s history and the CHANGELOG are always current for
anyone reading the repo directly. **Tagging is separate and
throttled**: established 2026-07-19 after v0.7.0-v0.7.3 shipped in a
4-day burst (see `~/.claude/projects/.../memory/` for the session
that measured this) — that burst wasn't itself a problem (each tag
was a genuinely complete, coherently-scoped unit of work, not an
arbitrary slice; per-tag diff size was comparable to or larger than
historically slower-cadence releases), but four crates.io publishes /
GitHub Releases in four days is more update-notification noise than
downstream consumers want, even when every individual change was
sound.

**Default cadence: every 2 weeks** (max wait 13 days, average 7) from
the last tag, bundling everything merged to `main` since then into
one release PR + tag. Don't tag opportunistically just because a
feature or fix finished — let it sit in the unreleased CHANGELOG
section until the next scheduled cut, *unless* the escape hatch below
applies.

**Escape hatch**: an out-of-cadence tag is fine for a security fix, a
data-loss/correctness bug serious enough to want off the broken
version quickly, or whenever the user explicitly asks for an
immediate release regardless of reason. Cutting early is the user's
call, not something to infer on your own from "this seems important."

**Versioning within this cadence**: a new protocol/mode addition is
patch-level by this crate's own established convention (MSK144
shipped as `0.7.4`, not `0.8.0` — grep `CHANGELOG.md` **and**
`docs/historical/CHANGELOG-0.6-0.7.md`/`CHANGELOG-0.x.md` for prior
protocol additions before assuming otherwise; older releases get
split out of the top-level file periodically to keep it skimmable,
see `CHANGELOG.md`'s own footer for the current archive list).
Minor bumps
(`0.6→0.7`) have historically marked a more structural change — a
breaking or structural API move, e.g. `0.7.0`'s generic
`decode_frame_for::<P>` API landing alongside FST4's remaining
sub-modes, or `0.10.0` collecting three public search-parameter type
changes — not simply "a release with new capability in it". When
genuinely unsure which a given accumulated batch warrants, ask rather
than default to whichever bump feels more exciting.

## Branching — `main` is trunk, `devel` is for open-ended experiments

Established 2026-07-25. Default workflow is unchanged: PRs land on
`main` immediately once they're a complete, coherently-scoped unit
(see release cadence above) — this is what keeps `main` safe for
downstream consumers who git-dependency-pin `branch = "main"` in
their `Cargo.toml`.

`devel` exists as a holding branch for work where the outcome isn't
known yet — embedded bring-up experiments, algorithm changes chasing
a numerical gap (e.g. JT65-style sensitivity work) — anything that
might get reverted rather than merged. Land commits there directly;
once an experiment resolves, PR the result into `main` as usual (or
just let the branch die if it didn't pan out).

If `devel` runs long, periodically merge/rebase `main` into it so it
doesn't rot into an unmergeable state by the time the experiment
concludes.

## House conventions

- **Cite the upstream.** Every algorithm file names the WSJT-X source it
  ports. Changing a decoder internal means reading the Fortran first,
  making the matching change, and keeping the `Ported from…` pointer
  valid. A deliberate divergence from WSJT-X is welcome but must be
  called out in a comment, so the next reader doesn't assume the Rust is
  still faithful.
- **Comments carry evidence, not intent.** The convention throughout this
  tree — `Cargo.toml` feature docs, `wspr::decode`'s cycle-budget table,
  `process_candidates.rs`'s LLR-format history — is that a knob's doc
  says what was measured and on what corpus. Match it; a new constant
  wants the number that chose it.
- **Commit messages**: short imperative subject (`fix: FST4 sync pattern
  off-by-one`), blank line, then a paragraph on the *why* with
  cross-references — issue number, and the upstream SHA / `file:line` if
  the change came from WSJT-X.
- **CHANGELOG entries land with the PR**, in the top section, written for
  someone who wasn't in the conversation.
- **Bilingual docs**: `docs/reference/*.md` all have `.ja.md` twins. Keep
  them in step or say which one you skipped.
- **Don't hardcode machine paths** anywhere a test can reach — see "Test
  fixture paths".

## Memory

- `~/.claude/projects/-home-minoru-src-mfsk-core/memory/` holds the
  per-conversation auto-memory. `project_decode_block_embedded.md` is the
  authoritative log of the embedded-port performance journey — read it
  before touching `decode_block` or any of the production app crates
  (`m5stack-s3-app`, `m5stack-core2-app`, `m5stack-cores3-app`).
