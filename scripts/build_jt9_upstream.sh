#!/usr/bin/env bash
# Build the upstream `jt9` and `wsprd` the upstream baseline is measured
# against (docs/notes/upstream/, scripts/upstream-baseline.py).
#
# Usage:
#   scripts/build_jt9_upstream.sh [WSJT-X-repo] [tag]
#     WSJT-X-repo  a clone that has the tag   (default ../WSJT-X)
#     tag          the release to build       (default v3.2.0-rc1)
#
# Output: target/upstream/build-<tag>/{jt9,wsprd}, and their sha256 on stdout.
# The source is exported with `git archive`, so the clone is not touched.
#
# WHY A SCRIPT
#
# The upstream baseline is only reproducible if the binary is. The one used
# for FT8_BENCHMARK.md §13 (2026-09-24) was built by hand in a temporary
# directory, and the edits it needed were recorded in prose only. When it
# was needed again it had to be rebuilt from that prose. These are those
# edits. None of them touches a decoder: each only drops a component that
# jt9 and wsprd do not link and that this host cannot configure.
#   - Qt5::WebSockets, used by the GUI and the TCI test simulator only,
#     is not installed here; dropped from find_package and wsjt_qt's link.
#   - map65 needs PortAudio, also not installed; gated behind
#     WSJT_SKIP_MAP65, including its install rule.
#   - docs and manpages need asciidoctor / a2x; turned off by upstream's own
#     options.
# CMake still stops at tests/unit/tci (it wants WebSockets), after it has
# written the Makefiles jt9 and wsprd need, so its exit status is ignored
# and the build's is checked instead.
#
# Check the result against the known counts: `jt9 -8 -d1/-d2/-d3` on
# embedded-poc/assets/qso3_busy.wav decodes 14 / 20 / 21 with v3.2.0-rc1.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WSJTX="${1:-$REPO_ROOT/../WSJT-X}"
TAG="${2:-v3.2.0-rc1}"

OUT="$REPO_ROOT/target/upstream"
SRC="$OUT/wsjtx-$TAG"
BUILD="$OUT/build-$TAG"
mkdir -p "$OUT"
rm -rf "$SRC" "$BUILD"
mkdir -p "$SRC" "$BUILD"

git -C "$WSJTX" archive "$TAG" | tar -x -C "$SRC"
echo "source: $TAG = $(git -C "$WSJTX" rev-parse "$TAG^{commit}")"

sed -i 's/ Qt5::WebSockets//' "$SRC/CMakeLists.txt"
sed -i 's/ LinguistTools WebSockets REQUIRED/ LinguistTools REQUIRED/' "$SRC/CMake/Dependencies.cmake"
python3 - "$SRC" <<'PY'
import sys
src = sys.argv[1]
def patch(path, old, new):
    s = open(path).read()
    if old not in s:
        sys.exit(f"{path}: expected text not found; this script targets v3.2.0-rc1")
    open(path, "w").write(s.replace(old, new, 1))
patch(f"{src}/CMakeLists.txt",
      "find_package (Portaudio REQUIRED)\nadd_subdirectory (map65)",
      "if (NOT WSJT_SKIP_MAP65)\nfind_package (Portaudio REQUIRED)\nadd_subdirectory (map65)\nendif ()")
old = ("install (TARGETS map65\n"
       "  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime\n"
       "  BUNDLE DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime\n"
       "  )")
patch(f"{src}/CMake/Install.cmake", old, "if (TARGET map65)\n" + old + "\nendif ()")
PY

( cd "$BUILD" && cmake -DCMAKE_BUILD_TYPE=Release -DWSJT_SKIP_MAP65=ON -DWSJT_SKIP_QMAP=ON \
    -DWSJT_GENERATE_DOCS=OFF -DWSJT_SKIP_MANPAGES=ON "$SRC" > cmake.log 2>&1 ) || true
( cd "$BUILD" && make -j"$(nproc)" jt9 wsprd > make.log 2>&1 ) || {
  echo "error: build failed, see $BUILD/make.log" >&2
  exit 1
}
sha256sum "$BUILD/jt9" "$BUILD/wsprd"
