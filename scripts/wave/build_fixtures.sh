#!/usr/bin/env bash
# Vendored TX waveforms for mfsk-core/tests/gen_wave_vs_wsjtx.rs: WSJT-X's own gen_ft8wave and gen_ft4wave
# (lib/ft8/gen_ft8wave.f90, lib/ft4/gen_ft4wave.f90) on a fixed tone sequence, from a clean `git archive` of a tag.
#
#   scripts/wave/build_fixtures.sh [WSJT-X-dir] [out-dir]
#
# Defaults: ../WSJT-X, mfsk-core/tests/fixtures/gen_wave. WAVE_TAG defaults to v3.3.0-beta6.
# Requires gfortran, git, tar. Output: ft8.tones ft8.i16 ft4.tones ft4.i16 (+ TAG, the exported commit).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
TAG="${WAVE_TAG:-v3.3.0-beta6}"
WSJTX_DIR="${1:-$REPO_ROOT/../WSJT-X}"
OUT_DIR="$(realpath -m "${2:-$REPO_ROOT/mfsk-core/tests/fixtures/gen_wave}")"
git -C "$WSJTX_DIR" rev-parse --verify --quiet "$TAG^{commit}" >/dev/null \
  || { echo "tag $TAG not found in $WSJTX_DIR" >&2; exit 1; }
WORK="$(mktemp -d)"; trap 'rm -rf "$WORK"' EXIT
git -C "$WSJTX_DIR" archive "$TAG" lib | tar -x -C "$WORK"
LIB="$WORK/lib"
mkdir -p "$OUT_DIR"
(cd "$WORK" && gfortran -O2 -w -fmax-array-constructor=70000 -o "$WORK/oracle" \
  "$LIB/timer_module.f90" "$LIB/gfsk_pulse.f90" "$LIB/waveform_kernels.f90" \
  "$LIB/ft8/gen_ft8wave.f90" "$LIB/ft4/gen_ft4wave.f90" "$HERE/gen_wave_oracle.f90")
(cd "$OUT_DIR" && "$WORK/oracle")
printf '%s %s\n' "$TAG" "$(git -C "$WSJTX_DIR" rev-parse "$TAG^{commit}")" > "$OUT_DIR/TAG"
echo "wrote $OUT_DIR ($(cat "$OUT_DIR/TAG"))"
