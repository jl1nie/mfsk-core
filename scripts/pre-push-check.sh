#!/usr/bin/env bash
# Pre-push quality gate — mirrors what CI's rustfmt+clippy job checks.
# Install once with:  scripts/install-hooks.sh
set -euo pipefail

echo "► cargo fmt --check"
cargo fmt --check

echo "► cargo clippy (full features, tests)"
cargo clippy --workspace --all-targets --features full --no-deps -- \
  -D warnings -D clippy::perf

echo "✓ pre-push checks passed"
