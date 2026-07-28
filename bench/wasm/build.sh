#!/usr/bin/env bash
# Build the mfsk-wasm-bench harness for wasm32-unknown-unknown, Node target.
#
# Usage:
#   ./build.sh                 # +simd128 (from .cargo/config.toml)
#   ./build.sh --no-simd128    # scalar baseline, for the before/after comparison
#   ./build.sh --profiling     # keep wasm name section for `node --prof` (Stage C)
#
# Requires: wasm-pack, the wasm32-unknown-unknown rustup target,
# wasm-objdump (from the WABT toolkit) for the verification line below.
set -euo pipefail
cd "$(dirname "$0")"

WASM_PACK_ARGS=(--target nodejs)
RUSTFLAGS_OVERRIDE=()
ENV_OVERRIDE=()

for arg in "$@"; do
  case "$arg" in
    --no-simd128)
      # Empty RUSTFLAGS env var wins over target.*.rustflags in
      # .cargo/config.toml (Cargo's documented precedence), which is how
      # this disables the checked-in +simd128 default for comparison
      # builds. Verified via the wasm-objdump line below on every build —
      # don't trust this flag silently, check the printed count.
      RUSTFLAGS_OVERRIDE=(RUSTFLAGS=)
      ;;
    --profiling)
      # --profiling alone still strips the wasm "name" custom section
      # (wasm-opt's default; see [package.metadata.wasm-pack.profile.profiling]
      # in Cargo.toml for the -g override that keeps it) — but rustc also
      # needs to be told to emit DWARF/name info in the first place, which
      # a plain release build doesn't. Without both, `node --prof-process`
      # resolves everything as `wasm-function[N]` instead of real symbols.
      WASM_PACK_ARGS=(--target nodejs --profiling)
      ENV_OVERRIDE+=(CARGO_PROFILE_RELEASE_DEBUG=2)
      ;;
    *)
      echo "unknown arg: $arg" >&2
      exit 1
      ;;
  esac
done

env "${RUSTFLAGS_OVERRIDE[@]}" "${ENV_OVERRIDE[@]}" wasm-pack build "${WASM_PACK_ARGS[@]}"

echo -n "v128/f32x4/i32x4 instruction count: "
wasm-objdump -d pkg/mfsk_wasm_bench_bg.wasm | grep -cE 'v128|f32x4|i32x4' || true
