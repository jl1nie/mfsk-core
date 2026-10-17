#!/usr/bin/env bash
# Build jt72_oracle (scripts/jt72/jt72_oracle.f90) against a clean export of a WSJT-X tag:
# packjt (packmsg / unpackmsg) and what it uses, compiled from `git archive`, so local edits under
# ../WSJT-X/lib cannot leak in. Same shape as scripts/pack77/build_oracle.sh.
#
#   scripts/jt72/build_oracle.sh [WSJT-X-dir] [out-dir]
#
# Defaults: ../WSJT-X, target/jt72-oracle. JT72_TAG defaults to v3.2.0-rc1.
# Requires gfortran, git, tar. Output: <out-dir>/jt72_oracle (+ exported-commit)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
TAG="${JT72_TAG:-v3.2.0-rc1}"
WSJTX_DIR="${1:-$REPO_ROOT/../WSJT-X}"
OUT_DIR="$(realpath -m "${2:-$REPO_ROOT/target/jt72-oracle}")"
git -C "$WSJTX_DIR" rev-parse --verify --quiet "$TAG^{commit}" >/dev/null \
  || { echo "tag $TAG not found in $WSJTX_DIR" >&2; exit 1; }
rm -rf "$OUT_DIR"; mkdir -p "$OUT_DIR/src" "$OUT_DIR/build"
git -C "$WSJTX_DIR" archive "$TAG" lib | tar -x -C "$OUT_DIR/src"
git -C "$WSJTX_DIR" rev-parse "$TAG^{commit}" > "$OUT_DIR/exported-commit"
LIB="$OUT_DIR/src/lib"
cd "$OUT_DIR/build"
F=(-O2 -w -I"$LIB")
gfortran "${F[@]}" -c "$LIB/packjt.f90"
for f in fmtmsg.f90 grid2deg.f90 deg2grid.f90; do
  gfortran "${F[@]}" -c "$LIB/$f"
done
gfortran "${F[@]}" *.o "$HERE/jt72_oracle.f90" -o "$OUT_DIR/jt72_oracle"
echo "built $OUT_DIR/jt72_oracle ($TAG $(cat "$OUT_DIR/exported-commit"))"
