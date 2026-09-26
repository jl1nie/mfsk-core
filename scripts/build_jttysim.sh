#!/usr/bin/env bash
# Build WSJT-X's JTTY command-line tools -- sjtty (simulator), rjtty
# (receiver) -- plus our own jtty_ladder_oracle driver, from a *clean export
# of a WSJT-X tag*.
#
# Usage:
#   scripts/build_jttysim.sh [WSJT-X-dir] [out-dir]
#
# Defaults:
#   WSJT-X-dir  ../WSJT-X  (sibling of this repo; only used as a git repo to
#               export the tag from -- its working tree is never read, so
#               local edits under lib/ cannot leak into the oracle)
#   out-dir     target/jttysim/
#   JTTY_TAG    v3.2.0-rc1 (environment override)
#
# Requires: gfortran, gcc, g++, cmake, libfftw3-dev (single + double), libboost-dev
#   Ubuntu:  sudo apt-get install gfortran cmake libfftw3-dev libboost-dev
#
# Why not upstream's own CMake: its top-level build requires Qt5 (incl.
# WebSockets), Hamlib and PortAudio to configure, none of which the JTTY
# Fortran programs use. scripts/jttysim/CMakeLists.txt replaces the exported
# root CMakeLists.txt and reuses upstream's CMake/Sources.cmake for the
# source lists, so the compile units are still upstream's own.
#
# Outputs: <out-dir>/build/{sjtty,rjtty,jtty_ladder_oracle}
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TAG="${JTTY_TAG:-v3.2.0-rc1}"

WSJTX_DIR="${1:-$(cd "$REPO_ROOT/../WSJT-X" 2>/dev/null && pwd || echo "")}"
OUT_DIR="${2:-$REPO_ROOT/target/jttysim}"

if [[ -z "$WSJTX_DIR" || ! -d "$WSJTX_DIR/.git" && ! -f "$WSJTX_DIR/.git" ]]; then
  echo "error: WSJT-X git checkout not found. Pass its path as the first argument." >&2
  echo "  $0 /path/to/WSJT-X" >&2
  exit 1
fi
for tool in gfortran cmake git tar; do
  if ! command -v "$tool" &>/dev/null; then
    echo "error: $tool not found (see the Requires line at the top of this script)." >&2
    exit 1
  fi
done
if ! git -C "$WSJTX_DIR" rev-parse --verify --quiet "$TAG^{commit}" >/dev/null; then
  echo "error: tag $TAG not found in $WSJTX_DIR (try: git -C $WSJTX_DIR fetch --tags)." >&2
  exit 1
fi

COMMIT="$(git -C "$WSJTX_DIR" rev-parse "$TAG^{commit}")"
SRC="$OUT_DIR/src"
BUILD="$OUT_DIR/build"

if [[ "$(cat "$OUT_DIR/exported-commit" 2>/dev/null || true)" != "$COMMIT" ]]; then
  echo "Exporting $TAG ($COMMIT) to $SRC ..."
  rm -rf "$SRC" "$BUILD"
  mkdir -p "$SRC"
  git -C "$WSJTX_DIR" archive "$TAG" | tar -x -C "$SRC"
  mv "$SRC/CMakeLists.txt" "$SRC/CMakeLists.upstream.txt"
  cp "$SCRIPT_DIR/jttysim/CMakeLists.txt" "$SRC/CMakeLists.txt"
  echo "$COMMIT" > "$OUT_DIR/exported-commit"
else
  echo "Reusing the export of $TAG ($COMMIT) in $SRC"
  cp "$SCRIPT_DIR/jttysim/CMakeLists.txt" "$SRC/CMakeLists.txt"
fi

cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
      -DJTTYSIM_DIR="$SCRIPT_DIR/jttysim" > "$OUT_DIR/configure.log" 2>&1 \
  || { echo "cmake configure failed; see $OUT_DIR/configure.log" >&2; tail -20 "$OUT_DIR/configure.log" >&2; exit 1; }
cmake --build "$BUILD" --target sjtty rjtty jtty_ladder_oracle -j"$(nproc)" \
      > "$OUT_DIR/build.log" 2>&1 \
  || { echo "build failed; see $OUT_DIR/build.log" >&2; grep -n -i error "$OUT_DIR/build.log" | head -20 >&2; exit 1; }

echo "Built from $TAG ($COMMIT):"
ls -1 "$BUILD/sjtty" "$BUILD/rjtty" "$BUILD/jtty_ladder_oracle"
