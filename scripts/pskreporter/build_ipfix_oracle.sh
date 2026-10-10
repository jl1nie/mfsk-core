#!/usr/bin/env bash
# Build scripts/pskreporter/ipfix_oracle.cpp against WSJT-X's own PSKReporterIPFIX.cpp.
#
#   scripts/pskreporter/build_ipfix_oracle.sh [WSJT-X-repo] [tag]     (defaults ../WSJT-X, v3.3.0-beta1)
#
# Output: target/pskreporter/ipfix_oracle. Needs Qt5Core's development files and g++; nothing else
# (upstream's Radio.hpp pulls in more Qt than this needs, so a stub with Radio::Frequency stands in).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WSJTX="${1:-$REPO_ROOT/../WSJT-X}"
TAG="${2:-v3.3.0-beta1}"
OUT="$REPO_ROOT/target/pskreporter"
rm -rf "$OUT"; mkdir -p "$OUT/src/Network" "$OUT/stub"
git -C "$WSJTX" show "$TAG:Network/PSKReporterIPFIX.cpp" > "$OUT/src/Network/PSKReporterIPFIX.cpp"
git -C "$WSJTX" show "$TAG:Network/PSKReporterIPFIX.hpp" > "$OUT/src/Network/PSKReporterIPFIX.hpp"
cat > "$OUT/stub/Radio.hpp" <<'STUB'
#pragma once
#include <QtGlobal>
namespace Radio { using Frequency = quint64; }
STUB
FLAGS=$(pkg-config --cflags --libs Qt5Core)
g++ -std=c++17 -O1 -fPIC -I"$OUT/stub" -I"$OUT/src" -I"$OUT/src/Network" \
    "$OUT/src/Network/PSKReporterIPFIX.cpp" "$SCRIPT_DIR/ipfix_oracle.cpp" $FLAGS -o "$OUT/ipfix_oracle"
echo "built from $TAG ($(git -C "$WSJTX" rev-parse --short "$TAG^{commit}")): $OUT/ipfix_oracle"
