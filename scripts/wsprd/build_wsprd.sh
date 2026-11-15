#!/usr/bin/env bash
# Build upstream `wsprd` from a clean export of a WSJT-X tag, without CMake/Qt: the sources of
# `add_executable(wsprd ...)` (CMakeLists.txt:550-554, CMake/Sources.cmake:737-748 `wsprd_CSRCS`)
# compiled with the flags a default (RELEASE) CMake build of that tag gives them on GCC
# (CMake's -O3 -DNDEBUG, CMake/CompilerFlags.cmake:11,23,36 for C; :44,69 for Fortran).
# Then it runs upstream's own golden check (tests/integration/wsprd/compare_wsprd_output.cmake:
# `wsprd -a <dir> samples/WSPR/150426_0918.wav` byte-equal to test/fixtures/wspr_150426_0918.expected.txt),
# so a binary that differs from upstream's is refused before it produces a single fixture.
#
#   scripts/wsprd/build_wsprd.sh [WSJT-X-dir] [out-dir]
#
# Defaults: ../WSJT-X, target/wsprd-<tag>. WSPRD_TAG defaults to v3.3.0-beta6.
# Requires gcc, gfortran, libfftw3-dev (fftw3f), git, tar.
# Output: <out-dir>/wsprd (+ exported-commit). The clone is not touched (`git archive`).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
TAG="${WSPRD_TAG:-v3.3.0-beta6}"
WSJTX_DIR="${1:-$REPO_ROOT/../WSJT-X}"
OUT_DIR="$(realpath -m "${2:-$REPO_ROOT/target/wsprd-$TAG}")"
for t in gcc gfortran git tar; do
  command -v "$t" >/dev/null || { echo "error: $t not found" >&2; exit 1; }
done
[[ -f /usr/include/fftw3.h ]] || { echo "error: fftw3.h not found (apt-get install libfftw3-dev)" >&2; exit 1; }
git -C "$WSJTX_DIR" rev-parse --verify --quiet "$TAG^{commit}" >/dev/null \
  || { echo "tag $TAG not found in $WSJTX_DIR" >&2; exit 1; }
rm -rf "$OUT_DIR"; mkdir -p "$OUT_DIR/src" "$OUT_DIR/obj"
git -C "$WSJTX_DIR" archive "$TAG" lib | tar -x -C "$OUT_DIR/src"
git -C "$WSJTX_DIR" rev-parse "$TAG^{commit}" > "$OUT_DIR/exported-commit"
S="$OUT_DIR/src"
CFLAGS=(-O3 -DNDEBUG -Wall -Wextra -fdata-sections -ffunction-sections -pthread -w -I"$S/lib" -I"$S/lib/wsprd")
FFLAGS=(-O3 -DNDEBUG -funroll-loops -fno-f2c -ffpe-summary=invalid,zero,overflow,underflow
        -Wall -Wno-conversion -Wno-c-binding-type -fno-second-underscore -w)
cd "$OUT_DIR/obj"
for c in lib/lookup3.c lib/wsprd/wsprd.c lib/wsprd/wsprsim_utils.c lib/wsprd/wsprd_utils.c \
         lib/wsprd/fano.c lib/wsprd/jelinek.c lib/wsprd/tab.c lib/wsprd/nhash.c \
         lib/init_random_seed.c lib/wsprd/wsprd_stream.c; do
  gcc "${CFLAGS[@]}" -c "$S/$c" -o "$(basename "$c" .c).o"
done
for f in lib/indexx.f90 lib/wsprd/osdwspr.f90; do
  gfortran "${FFLAGS[@]}" -c "$S/$f" -o "$(basename "$f" .f90).o"
done
gfortran -o "$OUT_DIR/wsprd" ./*.o -lfftw3f -lm -pthread
echo "built $OUT_DIR/wsprd ($TAG $(cat "$OUT_DIR/exported-commit"))"

# Upstream's own golden, needs the clone's samples/ and test/fixtures/ (not in `git archive lib`).
SAMPLE="$WSJTX_DIR/samples/WSPR/150426_0918.wav"
EXPECTED_REL="test/fixtures/wspr_150426_0918.expected.txt"
if [[ -f "$SAMPLE" ]]; then
  W="$OUT_DIR/golden"; mkdir -p "$W"
  git -C "$WSJTX_DIR" show "$TAG:$EXPECTED_REL" > "$W/expected.txt"
  ( cd "$W" && "$OUT_DIR/wsprd" -a "$W" "$SAMPLE" > actual.txt 2> stderr.txt )
  if [[ -s "$W/stderr.txt" ]] || ! cmp -s "$W/expected.txt" "$W/actual.txt"; then
    echo "error: wsprd differs from upstream's $EXPECTED_REL (see $W)" >&2; exit 1
  fi
  echo "upstream golden ($EXPECTED_REL): identical"
else
  echo "warning: $SAMPLE not found; upstream golden check skipped" >&2
fi
