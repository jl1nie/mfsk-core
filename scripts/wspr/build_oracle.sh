#!/usr/bin/env bash
# Build wspr_pack_oracle (scripts/wspr/wspr_pack_oracle.f90) against a clean export of a WSJT-X tag:
# wqencode / wqdecode and what they use, compiled from `git archive`, so local edits under
# ../WSJT-X/lib cannot leak in. Same shape as scripts/pack77/build_oracle.sh.
#
#   scripts/wspr/build_oracle.sh [WSJT-X-dir] [out-dir]
#
# Defaults: ../WSJT-X, target/wspr-oracle. WSPR_TAG defaults to v3.2.0-rc1.
# Two builds of the same sources, both -O0 -fcheck=bounds -finit-local-zero:
#   wspr_pack_oracle     -finit-character=0                          (the reference)
#   wspr_pack_oracle_b   -finit-integer=12345 -finit-character=65
# A message whose output differs between the two reads memory wqencode never wrote (it drops the
# `text` flag of packcall, and hashes `len` bytes of a 12-byte buffer): gen_cases.sh marks it.
# Requires gfortran, gcc, git, tar. Output: <out-dir>/wspr_pack_oracle{,_b} (+ exported-commit)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
TAG="${WSPR_TAG:-v3.2.0-rc1}"
WSJTX_DIR="${1:-$REPO_ROOT/../WSJT-X}"
OUT_DIR="$(realpath -m "${2:-$REPO_ROOT/target/wspr-oracle}")"
git -C "$WSJTX_DIR" rev-parse --verify --quiet "$TAG^{commit}" >/dev/null \
  || { echo "tag $TAG not found in $WSJTX_DIR" >&2; exit 1; }
rm -rf "$OUT_DIR"; mkdir -p "$OUT_DIR/src"
git -C "$WSJTX_DIR" archive "$TAG" lib | tar -x -C "$OUT_DIR/src"
git -C "$WSJTX_DIR" rev-parse "$TAG^{commit}" > "$OUT_DIR/exported-commit"
LIB="$OUT_DIR/src/lib"
build() { # name, extra gfortran flags...
  local name="$1"; shift
  mkdir -p "$OUT_DIR/build_$name"; cd "$OUT_DIR/build_$name"
  local F=(-O0 -w -I"$LIB" -fcheck=bounds -finit-local-zero "$@")
  gcc -O0 -w -I"$LIB" -I"$LIB/wsprd" -c "$LIB/lookup3.c" "$LIB/wsprd/nhash.c"
  gfortran "${F[@]}" -c "$LIB/hashing.f90" "$LIB/packjt.f90"
  gfortran "${F[@]}" -c "$LIB/hash.f90" "$LIB/wqencode.f90" "$LIB/wqdecode.f90" \
    "$LIB/grid2deg.f90" "$LIB/deg2grid.f90" "$LIB/fmtmsg.f90"
  gfortran "${F[@]}" *.o "$HERE/wspr_pack_oracle.f90" -o "$OUT_DIR/$name"
}
build wspr_pack_oracle -finit-character=0
build wspr_pack_oracle_b -finit-integer=12345 -finit-character=65
echo "built $OUT_DIR/wspr_pack_oracle{,_b} ($TAG $(cat "$OUT_DIR/exported-commit"))"
