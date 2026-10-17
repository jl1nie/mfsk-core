#!/usr/bin/env bash
# Build pack77_oracle (scripts/pack77/pack77_oracle.f90) against a clean export of a WSJT-X tag:
# the 77-bit packer packjt77 and what it uses, compiled from `git archive`, so local edits under
# ../WSJT-X/lib cannot leak in. Same shape as scripts/fst4w/build_oracle.sh.
#
#   scripts/pack77/build_oracle.sh [WSJT-X-dir] [out-dir]
#
# Defaults: ../WSJT-X, target/pack77-oracle. PACK77_TAG defaults to v3.2.0-rc1.
# Requires gfortran, git, tar. Output: <out-dir>/pack77_oracle (+ exported-commit)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
TAG="${PACK77_TAG:-v3.2.0-rc1}"
WSJTX_DIR="${1:-$REPO_ROOT/../WSJT-X}"
OUT_DIR="$(realpath -m "${2:-$REPO_ROOT/target/pack77-oracle}")"
git -C "$WSJTX_DIR" rev-parse --verify --quiet "$TAG^{commit}" >/dev/null \
  || { echo "tag $TAG not found in $WSJTX_DIR" >&2; exit 1; }
rm -rf "$OUT_DIR"; mkdir -p "$OUT_DIR/src" "$OUT_DIR/build"
git -C "$WSJTX_DIR" archive "$TAG" lib | tar -x -C "$OUT_DIR/src"
git -C "$WSJTX_DIR" rev-parse "$TAG^{commit}" > "$OUT_DIR/exported-commit"
LIB="$OUT_DIR/src/lib"
cd "$OUT_DIR/build"
F=(-O2 -w -I"$LIB")
for f in chkcall.f90 77bit/packjt77_grammar.f90 77bit/packjt77_schema.f90 77bit/packjt77.f90; do
  gfortran "${F[@]}" -c "$LIB/$f"
done
gfortran "${F[@]}" *.o "$HERE/pack77_oracle.f90" -o "$OUT_DIR/pack77_oracle"
echo "built $OUT_DIR/pack77_oracle ($TAG $(cat "$OUT_DIR/exported-commit"))"
