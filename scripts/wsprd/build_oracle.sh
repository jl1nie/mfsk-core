#!/usr/bin/env bash
# Build wsprd_unpack_oracle (scripts/wsprd/wsprd_unpack_oracle.c) against a clean export of a WSJT-X
# tag: lib/wsprd/wsprd_utils.c (unpk_ and what it calls), nhash.c, lib/lookup3.c, compiled from
# `git archive`, so local edits under ../WSJT-X cannot leak in. Same shape as scripts/wspr/build_oracle.sh.
#
#   scripts/wsprd/build_oracle.sh [WSJT-X-dir] [out-dir]
#
# Defaults: ../WSJT-X, target/wsprd-unpack-oracle. WSPRD_TAG defaults to v3.2.0-rc1.
# Requires gcc, git, tar. Output: <out-dir>/wsprd_unpack_oracle (+ exported-commit)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
TAG="${WSPRD_TAG:-v3.2.0-rc1}"
WSJTX_DIR="${1:-$REPO_ROOT/../WSJT-X}"
OUT_DIR="$(realpath -m "${2:-$REPO_ROOT/target/wsprd-unpack-oracle}")"
git -C "$WSJTX_DIR" rev-parse --verify --quiet "$TAG^{commit}" >/dev/null \
  || { echo "tag $TAG not found in $WSJTX_DIR" >&2; exit 1; }
rm -rf "$OUT_DIR"; mkdir -p "$OUT_DIR/src"
git -C "$WSJTX_DIR" archive "$TAG" lib | tar -x -C "$OUT_DIR/src"
git -C "$WSJTX_DIR" rev-parse "$TAG^{commit}" > "$OUT_DIR/exported-commit"
LIB="$OUT_DIR/src/lib"
gcc -O0 -w -I"$LIB" -I"$LIB/wsprd" \
  "$HERE/wsprd_unpack_oracle.c" "$LIB/wsprd/wsprd_utils.c" "$LIB/wsprd/nhash.c" "$LIB/lookup3.c" \
  -o "$OUT_DIR/wsprd_unpack_oracle"
echo "built $OUT_DIR/wsprd_unpack_oracle ($TAG $(cat "$OUT_DIR/exported-commit"))"
