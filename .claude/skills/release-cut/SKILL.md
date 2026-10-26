---
name: release-cut
description: Cut an mfsk-core library release (vX.Y.Z) or a skimmer release (skimmer-vX.Y.Z) - release-status check, release PR, tier-C sweeps, tag. Use when the user asks to prepare, cut or tag a release.
disable-model-invocation: true
---

# Cutting a release

The procedure of record is `docs/notes/RELEASE_PROCEDURE.md`; this skill is the order of operations and
the points where releases went wrong before. Read that file for anything not covered here.

## 0. Compute the state, don't recall it

```sh
scripts/release-status.sh
```

Quote its output. It says: whether the workspace version is tagged, whether CHANGELOG's top section matches,
the cadence (target 13-14 days since the last regular tag), and which protocols' sweeps are outstanding, with
the commit subjects. Do not take any release-state claim from memory, CLAUDE.md or the procedure note.

Stop and ask the user if:
- the cadence is short of ~13 days (the escape hatch - security, data loss, explicit request - is the user's call);
- patch vs minor is unclear (new protocol = patch; structural / breaking API = minor; grep `CHANGELOG.md` and
  `docs/historical/CHANGELOG-*.md` for precedent first).

## 1. Release PR - metadata and docs only

Behaviour fixes land first, in their own PRs. The release PR carries:
- `workspace.package.version` in the root `Cargo.toml` (the single version; `mfsk-ffi*` inherit it and take
  `mfsk-core` by path), then `cargo check` so `Cargo.lock` follows;
- the CHANGELOG section heading/date; at a new minor, move the oldest minor series to `docs/historical/`;
- docs that name the version, in both `docs/reference/*.md` and their `.ja.md` twins;
- at most one process change proposed from the `AGENT_LOG.md` summary `release-status.sh` printed
  (`docs/notes/AGENT_WORKFLOW.md` §7).

`scripts/verify.sh` before merging (last line `VERIFY PASS`), `scripts/ci-wait.sh` after the push.

Skimmer release instead: its own version in `apps/skimmer/{core,cli}/Cargo.toml`,
`apps/skimmer/gui/src-tauri/Cargo.toml`, `apps/skimmer/gui/src-tauri/tauri.conf.json` (checked by
`skimmer-release.yml`) and `apps/skimmer/gui/package.json`, all agreeing. The skimmer takes `mfsk-core` by
path, so it ships `main` at that commit.

## 2. After the PR merges

```sh
git checkout main && git pull
scripts/run-sensitivity-sweeps.sh <modes release-status.sh named>
```

Read the `sweep-regression-check.py` table at the end: `!!` on a 50%-crossing move >= 0.5 dB or on a
precision rise. Explain any `!!` before tagging. Refresh the baseline only once the move is understood
(`python3 scripts/sweep-regression-check.py --update-baseline target/sweep-csv/*.csv`); never hand-edit
`_meta.protocols`.

## 3. Tag - confirm with the user first

```sh
git tag vX.Y.Z <merge-sha> && git push origin vX.Y.Z      # or skimmer-vX.Y.Z
```

The PreToolUse hook asks before a tag push. Never `cargo publish` (denied): `release.yml` waits for CI and the
tier A+B job, then publishes. Watch the `Release` workflow with `gh run watch`.

## 4. Report

Announcement angle (when asked to draft one): WSJT-X parity, speed, what the API change buys; library and app
posts separate; check that a fix applies to the product before claiming it.
