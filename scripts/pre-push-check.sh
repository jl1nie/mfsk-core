#!/usr/bin/env bash
# Pre-push quality gate — mirrors what CI's rustfmt+clippy, rustdoc, and
# feature-matrix jobs check.
# Install once with:  scripts/install-hooks.sh
#
# The feature-matrix and rustdoc steps were added after PR #233 (2026-08)
# shipped code that only compiled clean under `--features full` and
# `--features full,internal-testing` — CI's per-protocol build matrix
# (`--no-default-features` plus each single protocol feature) caught a
# `dead_code` error and rustdoc caught broken intra-doc links that
# neither `cargo fmt` nor `cargo clippy --features full,internal-testing`
# ever exercised. `dead_code` in particular depends on which cfg'd items
# are reachable for a *given* feature set — passing under one or two
# combos says nothing about the rest of this crate's feature matrix.
set -euo pipefail

echo "► cargo fmt --check"
cargo fmt --check

echo "► cargo clippy (full features, tests)"
cargo clippy --workspace --all-targets --features full,internal-testing --no-deps -- \
  -D warnings -D clippy::perf

echo "► cargo doc (full features)"
RUSTDOCFLAGS="-D warnings" cargo doc -p mfsk-core --no-deps --features full

echo "► feature matrix (mirrors ci.yml's feature-matrix job)"
FEATURE_MATRIX=(
  ""
  "ft8"
  "ft4"
  "fst4"
  "wspr"
  "jt9"
  "jt65"
  "q65"
  "uvpacket"
  "alloc ft8"
  "alloc ft8 fft-extern"
  "alloc ft8 fft-extern fixed-point"
  "alloc ft4 fft-extern"
  "alloc ft4 fft-extern fixed-point"
  "full"
)
for features in "${FEATURE_MATRIX[@]}"; do
  echo "  · [${features:-<none>}]"
  if [ -z "$features" ]; then
    RUSTFLAGS="-D warnings" cargo build -p mfsk-core --no-default-features --release
  else
    RUSTFLAGS="-D warnings" cargo build -p mfsk-core --no-default-features --features "$features" --release
  fi
done

echo "✓ pre-push checks passed"
