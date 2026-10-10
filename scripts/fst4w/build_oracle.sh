#!/usr/bin/env bash
# Build genfst4w_oracle (scripts/fst4w/genfst4w_oracle.f90) against a clean
# export of a WSJT-X tag: genfst4 and everything it needs, as
# scripts/build_fst4sim.sh compiles them, but from `git archive`, so local edits
# under ../WSJT-X/lib cannot leak in.
#
#   scripts/fst4w/build_oracle.sh [WSJT-X-dir] [out-dir]
#
# Defaults: ../WSJT-X, target/fst4w-oracle. FST4W_TAG defaults to v3.3.0-beta1.
# Requires gfortran, git, tar. Output: <out-dir>/genfst4w_oracle
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
TAG="${FST4W_TAG:-v3.3.0-beta1}"
WSJTX_DIR="${1:-$REPO_ROOT/../WSJT-X}"
OUT_DIR="$(realpath -m "${2:-$REPO_ROOT/target/fst4w-oracle}")"
git -C "$WSJTX_DIR" rev-parse --verify --quiet "$TAG^{commit}" >/dev/null \
  || { echo "tag $TAG not found in $WSJTX_DIR" >&2; exit 1; }
rm -rf "$OUT_DIR"; mkdir -p "$OUT_DIR/src" "$OUT_DIR/build"
git -C "$WSJTX_DIR" archive "$TAG" lib | tar -x -C "$OUT_DIR/src"
git -C "$WSJTX_DIR" rev-parse "$TAG^{commit}" > "$OUT_DIR/exported-commit"
LIB="$OUT_DIR/src/lib"
cd "$OUT_DIR/build"
F=(-O2 -w -I"$LIB/fst4" -I"$LIB")
for f in crc.f90 packjt.f90 77bit/packjt77_grammar.f90 77bit/packjt77_schema.f90 77bit/packjt77.f90 \
         deg2grid.f90 grid2deg.f90 fmtmsg.f90 chkcall.f90 fst4/get_crc24.f90 \
         fst4/encode240_101.f90 fst4/encode240_74.f90 fst4/genfst4.f90; do
  gfortran "${F[@]}" -c "$LIB/$f"
done
gfortran "${F[@]}" *.o "$HERE/genfst4w_oracle.f90" -o "$OUT_DIR/genfst4w_oracle"
echo "built $OUT_DIR/genfst4w_oracle ($TAG $(cat "$OUT_DIR/exported-commit"))"
