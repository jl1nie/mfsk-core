#!/usr/bin/env bash
# Regenerate mfsk-core/tests/fixtures/wsprd_beta6/: what upstream `wsprd` (built by build_wsprd.sh)
# writes for the vendored golden embedded-poc/assets/golden/wspr/150426_0918.wav, once per option set.
# Each case directory holds `args` (the options, one line), `stdout.txt`, `ALL_WSPR.TXT`,
# `wspr_spots.txt`, `hashtable.txt` (absent for -H, as wsprd writes none) and `exit`.
# wspr_timer.out and wspr_wisdom.dat are not kept: timings and FFTW plans, not results.
# Every case is run twice and refused unless both runs are byte-identical.
#
#   scripts/wsprd/gen_fixtures.sh [wsprd-binary]
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
WSPRD="$(realpath "${1:-$REPO_ROOT/target/wsprd-v3.3.0-beta6/wsprd}")"
WAV="$REPO_ROOT/embedded-poc/assets/golden/wspr/150426_0918.wav"
OUT="$REPO_ROOT/mfsk-core/tests/fixtures/wsprd_beta6"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
# name | options (wsprd's own spelling)
CASES=(
  "bare|"
  "f|-f 10.1387"
  "f_C500_o4|-f 10.1387 -C 500 -o 4"
  "f_C500_o4_d|-f 10.1387 -C 500 -o 4 -d"
  "f_qB|-f 10.1387 -qB"
  "f_s|-f 10.1387 -s"
  "f_w|-f 10.1387 -w"
  "f_z040|-f 10.1387 -z 0.40"
  "f_C100|-f 10.1387 -C 100"
  "f_o1|-f 10.1387 -o 1"
  "f_e60|-f 10.1387 -e 60"
  "f_H|-f 10.1387 -H"
)
run() { # dir, options...
  local d=$1; shift
  mkdir -p "$d"; cp "$WAV" "$d/150426_0918.wav"
  ( cd "$d" && set +e; "$WSPRD" -a . "$@" 150426_0918.wav > stdout.txt 2> stderr.txt; echo $? > exit )
  [[ -s "$d/stderr.txt" ]] && { echo "error: wsprd wrote to stderr in $d" >&2; exit 1; }
  return 0
}
rm -rf "$OUT"; mkdir -p "$OUT"
for c in "${CASES[@]}"; do
  name=${c%%|*}; opts=${c#*|}
  # shellcheck disable=SC2086 # options are split on purpose
  run "$TMP/$name.a" $opts; run "$TMP/$name.b" $opts
  mkdir -p "$OUT/$name"; printf '%s\n' "$opts" > "$OUT/$name/args"
  for f in stdout.txt ALL_WSPR.TXT wspr_spots.txt hashtable.txt exit; do
    if [[ -f "$TMP/$name.a/$f" ]]; then
      cmp -s "$TMP/$name.a/$f" "$TMP/$name.b/$f" || { echo "error: $name/$f not deterministic" >&2; exit 1; }
      cp "$TMP/$name.a/$f" "$OUT/$name/$f"
    fi
  done
  echo "$name: $(grep -vc DecodeFinished "$OUT/$name/stdout.txt") decodes [$opts]"
done
{ echo "wsprd $(cat "$(dirname "$WSPRD")/exported-commit" 2>/dev/null || echo unknown) (v3.3.0-beta6, scripts/wsprd/build_wsprd.sh)"
  echo "input embedded-poc/assets/golden/wspr/150426_0918.wav sha256 $(sha256sum "$WAV" | cut -d' ' -f1)"; } > "$OUT/SOURCE"
