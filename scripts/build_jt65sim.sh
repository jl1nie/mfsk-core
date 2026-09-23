#!/usr/bin/env bash
# Re-link jt65sim against this repo's deterministic sgran stub.
#
# Usage:
#   scripts/build_jt65sim.sh [wsjtx-build-dir] [out-dir]
#
# Defaults:
#   wsjtx-build-dir  ~/wsjtx-build
#   out-dir          target/jt65sim/
#
# Unlike ft8sim/ft4sim/fst4sim/jt9sim/wsprsim, jt65sim has no minimal
# standalone dependency closure worth picking out -- it links the whole
# `wsjt_fort` + `wsjt_cxx` libraries per WSJT-X's own CMakeLists.txt. So
# this script does not rebuild it from scratch; it reuses the CMake
# tree's libraries and only replaces one object. Configure and build
# that tree first (see docs/notes/BENCHMARKS.md, "Generating the tier-C
# corpora"):
#
#   cmake -S /path/to/WSJT-X -B ~/wsjtx-build -DCMAKE_BUILD_TYPE=Release \
#         -DWSJT_GENERATE_DOCS=OFF -DWSJT_SKIP_MANPAGES=ON
#   cmake --build ~/wsjtx-build --target jt65sim -j"$(nproc)"
#
# WHY RE-LINK AT ALL
#
# `libwsjt_fort.a` carries upstream's `sgran.o`, which seeds C `rand()`
# from /dev/urandom -- so CMake's own `jt65sim` writes a different noise
# realisation on every run and its corpus cannot be reproduced on another
# machine. Passing `sim_sgran_stub.o` as an explicit object resolves
# `sgran_` before the archive is searched, and since upstream's sgran.o
# defines nothing else, that member is never pulled in. Same binary
# otherwise. See scripts/sim_sgran_stub.c.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

BUILD_DIR="${1:-$HOME/wsjtx-build}"
OUT_DIR="${2:-$REPO_ROOT/target/jt65sim}"

LINK_TXT="$BUILD_DIR/CMakeFiles/jt65sim.dir/link.txt"
if [[ ! -f "$LINK_TXT" ]]; then
  echo "error: no CMake jt65sim target at $BUILD_DIR" >&2
  echo "Configure and build it first -- see this script's header." >&2
  exit 1
fi

mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"
BUILD="$OUT_DIR/build"
mkdir -p "$BUILD"

echo "Re-linking jt65sim from $BUILD_DIR ..."

(
  cd "$BUILD"
  gcc -O2 -c "$SCRIPT_DIR/sim_sgran_stub.c" -o sgran_stub.o
)

# The CMake link line, with our object inserted ahead of the archives and
# the output path redirected. Everything else is upstream's own command.
LINE="$(cat "$LINK_TXT")"
LINE="${LINE/-o jt65sim/-o $OUT_DIR/jt65sim}"
LINE="${LINE/libwsjt_fort.a/$BUILD/sgran_stub.o libwsjt_fort.a}"

( cd "$BUILD_DIR" && eval "$LINE" )

echo ""
echo "Built: $OUT_DIR/jt65sim"
echo "Run 'scripts/gen_jt65_sweep_wavs.sh $OUT_DIR/jt65sim' to generate the corpus."
