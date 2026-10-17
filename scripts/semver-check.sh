#!/usr/bin/env bash
# Public-API check of mfsk-core against a released tag (default: the latest vX.Y.Z), by cargo-semver-checks.
# usage: scripts/semver-check.sh [BASELINE_TAG]       needs: cargo install cargo-semver-checks --locked
# Every failure it lists must be in CHANGELOG.md's "Breaking changes at a glance" (and, if the version is still 0.x
# with a patch bump, it must not be there at all). Run it in the release PR, before the tag (RELEASE_PROCEDURE.md).
# `full` only: wspr-ddc and wspr-ddc-cascade are mutually exclusive, so --all-features would not build.
set -euo pipefail
cd "$(dirname "$0")/.."
base=${1:-$(git tag --list 'v[0-9]*' --sort=-v:refname | head -1)}
echo "baseline: $base"
cargo semver-checks --package mfsk-core --baseline-rev "$base" --only-explicit-features --features full
