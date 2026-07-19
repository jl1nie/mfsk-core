# mfsk-core — agent notes

Repo-level notes for assistants working in this tree. Anything specific to
a sub-crate lives in its own `CLAUDE.md` or `README.md`; this file is for
cross-crate workflow that's easy to forget between sessions.

## Embedded targets

The active production crates are `embedded-poc/m5stack-s3-app/` (S3
LX7, repositioned as **demo / acoustic-fallback** in the 2026-05-17
pivot — the StickS3 board can't do USB host),
`embedded-poc/m5stack-core2-app/` (Core2 LX6, wav_sim only — no USB
peripheral on classic ESP32), and the planned
`embedded-poc/m5stack-cores3-app/` (S3 LX7, **main UAC controller
target** — CoreS3 has AXP2101 + AW9523B for proper USB-OTG host mode;
see `docs/notes/ROADMAP.md` Phase B-Core). Each has its own `CLAUDE.md`
covering board-specific bring-up; this section captures the shared
workflow that's easy to forget between sessions.

- **Build & flash via `espflash`**, not host cargo. Both crates'
  `.cargo/config.toml` set `runner = "espflash flash --monitor"`, so the
  basic user workflow is:
  ```sh
  cd embedded-poc/m5stack-s3-app   # or m5stack-core2-app
  cargo run --release              # builds + flashes + opens serial monitor
  ```
  In practice, for capturing per-session logs to a file, use
  `embedded-poc/scripts/flash-monitor.sh` (see next section) instead of
  the bare runner.
- The `+esp` Rust toolchain (Xtensa fork, espup-installed) is selected
  by each crate's `rust-toolchain.toml`. Target triple per
  `.cargo/config.toml`: `xtensa-esp32-espidf` for Core2 (LX6),
  `xtensa-esp32s3-espidf` for S3 (LX7).
- `cargo check` from inside each crate validates code changes without
  flashing (~30-50 s with prebuilt esp-idf).
- Logs from the device land in `embedded-poc/<crate>/logs/` — user has
  been capturing per-session sweep output there.
- The user actively flashes both boards during embedded work; do NOT
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

1. `espflash monitor` defaults to `--before default-reset`, which pulses
   DTR/RTS and on S3 USB-OTG boards drops the chip into DOWNLOAD mode
   (`rst:0x15 USB_UART_CHIP_RESET … waiting for download`). The script
   passes `--before no-reset --after no-reset` so the just-flashed app
   keeps running.
2. Re-flashing the same ELF prints "Segment … has not changed, skipping
   write" and finishes in ~5 s. **That is not a successful flash** — the
   chip still runs the previous binary. Touch a source file or change a
   `log::info!` line to force a real rewrite, and expect ~15-25 s for a
   real factory-partition write.

```sh
source ~/export-esp.sh
cd embedded-poc/m5stack-s3-app   # or m5stack-s3, m5stack-core2-app
cargo build --release --bin <bin>
../scripts/flash-monitor.sh \
    target/<triple>/release/<bin> \
    logs/<bin>_<tag>_$(date +%Y-%m-%d).log \
    90    # capture seconds (optional, default 90; use ≥120 for fresh Core2 flashes — 1.3 MB binary takes ~55 s to write)
```

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
  reference recordings already live there.
- **Out-of-tree user-machine assets** (e.g. the full WSJT-X tarball):
  `option_env!("WSJTX_SAMPLES_DIR")` and skip cleanly when unset.
- **Diagnostic output paths** (test writes a WAV for human inspection):
  `/tmp/...` literals are fine — the human-in-the-loop step assumes a
  known location. Don't replace these with `tempfile`.

## Releases — go through CD, never `cargo publish` locally

`mfsk-core` ships to crates.io via `.github/workflows/release.yml`,
triggered by a `vX.Y.Z` tag push. The workflow gates the publish on
the CI for the same commit going green (`wait-for-ci` job), then
runs `cargo publish -p mfsk-core --features full` + builds the
`mfsk-ffi-ft8` FFI artifacts + creates the GitHub release with
attached tarballs.

**Do not `cargo publish` from a local clone.** crates.io publishes
are irreversible — once a version is up, you cannot unpublish
(yank exists but blocks new dependents while existing dependents
keep using the broken version). A local publish bypasses the CI
gate that this whole workflow exists to enforce; even if your
local SHA happens to be CI-green, future releases get sloppier
when "just publish locally" is in the playbook. Push the tag and
let CD do it.

Sequence:
1. Merge release PR into `main`.
2. `git checkout main && git pull`.
3. `git tag vX.Y.Z <merge-sha>` then `git push origin vX.Y.Z`.
4. Watch the Actions tab for the `Release` workflow.

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
shipped as `0.7.4`, not `0.8.0` — grep `CHANGELOG.md` for prior
protocol additions before assuming otherwise). Minor bumps
(`0.6→0.7`) have historically marked a more structural change (e.g.
`0.7.0`'s generic `decode_frame_for::<P>` API landing alongside FST4's
remaining sub-modes), not simply "a release with new capability in
it" — when genuinely unsure which a given accumulated batch warrants,
ask rather than default to whichever bump feels more exciting.

## Memory

- `~/.claude/projects/-home-minoru-src-mfsk-core/memory/` holds the
  per-conversation auto-memory. `project_decode_block_embedded.md` is the
  authoritative log of the embedded-port performance journey — read it
  before touching `decode_block` or either of the production app crates
  (`m5stack-s3-app`, `m5stack-core2-app`).
