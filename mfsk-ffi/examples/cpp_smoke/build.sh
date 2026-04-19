#!/usr/bin/env bash
# Build + run the C++ smoke test for mfsk-ffi.
set -euo pipefail

cd "$(dirname "$0")"
REPO_ROOT="$(cd ../../.. && pwd)"
MFSK_TARGET="$REPO_ROOT/target/release"

cargo build -p mfsk-ffi --release

g++ -std=c++17 \
    -I"$REPO_ROOT/mfsk-ffi/include" \
    main.cpp \
    -L"$MFSK_TARGET" -lmfsk \
    -pthread -ldl -lm \
    -Wl,-rpath,"$MFSK_TARGET" \
    -O2 -Wall -Wextra \
    -o cpp_smoke

./cpp_smoke
