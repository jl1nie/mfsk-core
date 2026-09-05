#!/usr/bin/env bash
# Pre-push quality gate — mirrors what CI's rustfmt+clippy, rustdoc, and
# feature-matrix jobs check.
# Runs automatically as `.githooks/pre-push`; enable with
#   git config core.hooksPath .githooks
# (or `bash scripts/install-hooks.sh`, which sets exactly that).
# Also safe to run by hand at any time.
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

echo "► FT8 recall floors under fixed-point (issue #359)"
# `fixed-point` implies `nstep-half` and is the numeric path every
# embedded build ships, but nothing routine ran it: the merge gate is
# `full,internal-testing`, so this drifted silently for three weeks
# (#280's lag window) before anyone measured it. Scoped to the FT8
# recall-floor tests, not the whole crate — a few seconds, not another
# full `cargo test`.
#
# Deliberately no `RUSTFLAGS="-D warnings"` here (unlike the build
# matrix below): `+fixed-point,internal-testing` alone already emits
# pre-existing `unused import: num_traits::Float` warnings in library
# code this change didn't touch, and turning those into failures here
# is a separate cleanup, not this issue's scope.
MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core --release --no-default-features \
  --features full,fixed-point,internal-testing \
  --test ft8_qso3_apoff_recall \
  --test ft8_qso3_apon_recall \
  --test ft8_decode_block_real_qso
# The `not(fft-rustfft)` sibling: the driver that actually ships
# (`decode_block_multipass` has two `#[cfg]`-gated bodies split on
# that flag — see `ft8_embedded_driver_recall.rs`'s module doc).
# `ft8_decode_block_streaming` stays out of this line: it has no
# `fixed-point` i16 FFT shim of its own and isn't fixed-point-specific
# (streaming-vs-batch consistency, unaffected by numeric path).
MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core --release --no-default-features \
  --features alloc,ft8,fft-extern,fixed-point,internal-testing \
  --test ft8_embedded_driver_recall

echo "✓ pre-push checks passed"
