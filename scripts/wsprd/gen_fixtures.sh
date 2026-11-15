#!/usr/bin/env bash
# Regenerate mfsk-core/tests/fixtures/wsprd_beta6/: what upstream `wsprd` (built by build_wsprd.sh)
# writes for the vendored golden embedded-poc/assets/golden/wspr/150426_0918.wav, once per option set.
# Each case directory holds `args` (the options, one line), `stdout.txt`, `ALL_WSPR.TXT`,
# `wspr_spots.txt`, `hashtable.txt` (absent for -H, as wsprd writes none) and `exit`.
# wspr_timer.out and wspr_wisdom.dat are not kept: timings and FFTW plans, not results.
# Every case is run twice and refused unless both runs are byte-identical.
#
# A case written `name|options|seed` starts from the `hashtable.txt` that case `seed` saved, kept
# beside it as `hashtable_in.txt`: wsprd loads `hashtable.txt` from its data directory, and its OSD
# (`-o`) accepts only a call that table holds, so a fresh directory (every case above the `pre`
# ones) never lets OSD accept anything.
#
#   scripts/wsprd/gen_fixtures.sh [wsprd-binary]
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
WSPRD="$(realpath "${1:-$REPO_ROOT/target/wsprd-v3.3.0-beta6/wsprd}")"
WAV="$REPO_ROOT/embedded-poc/assets/golden/wspr/150426_0918.wav"
OUT="$REPO_ROOT/mfsk-core/tests/fixtures/wsprd_beta6"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
# name | options (wsprd's own spelling) | seed case (optional)
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
  "f_B|-f 10.1387 -B"
  "f_q|-f 10.1387 -q"
  "f_e45|-f 10.1387 -e 45"
  "t3|-f 10.1387"
  "t3_long|-f 10.1387"
  "far_B_e10|-f 10.1387 -B -e -10"
  "far_e10|-f 10.1387 -e -10"
  "pre|-f 10.1387|f"
  "pre_o0|-f 10.1387 -o 0|f"
  "pre_o1|-f 10.1387 -o 1|f"
  "pre_o2|-f 10.1387 -o 2|f"
  "pre_o3|-f 10.1387 -o 3|f"
  "pre_o4|-f 10.1387 -o 4|f"
  "pre_o5|-f 10.1387 -o 5|f"
  "pre_C500_o4_d|-f 10.1387 -C 500 -o 4 -d|f"
  "pre_o4_s|-f 10.1387 -o 4 -s|f"
)
# The `t3*` cases run on a synthetic recording (a Type 3 whose call is not in the table, and a
# -27 dB W1AW beside it), written byte-identically by tests/common/wspr_t3.rs through an example
# so that no 2.9 MB WAV is vendored; the test rebuilds it in memory.
T3WAV="$TMP/250101_0000.wav"
(cd "$REPO_ROOT" && cargo run -q --release -p mfsk-core --example gen_wsprd_t3_wav \
   --features full,internal-testing -- "$T3WAV")
# `far*`: one strong call at f1 -126 Hz, outside the +/-110 Hz window until -e -10 moves it twice.
FARWAV="$TMP/far/250101_0002.wav"; mkdir -p "$TMP/far"
(cd "$REPO_ROOT" && cargo run -q --release -p mfsk-core --example gen_wsprd_t3_wav \
   --features full,internal-testing -- "$FARWAV" far)
run() { # dir, seed hashtable ('' for none), options...
  local d=$1 seed=$2; shift 2
  local wavsrc=$WAV
  [[ "$(basename "$d")" == t3* ]] && wavsrc=$T3WAV
  [[ "$(basename "$d")" == far* ]] && wavsrc=$FARWAV
  local wavname; wavname=$(basename "$wavsrc")
  mkdir -p "$d"; cp "$wavsrc" "$d/$wavname"
  if [[ -n "$seed" ]]; then cp "$seed" "$d/hashtable.txt"; fi
  ( cd "$d" && set +e; "$WSPRD" -a . "$@" "$wavname" > stdout.txt 2> stderr.txt; echo $? > exit )
  [[ -s "$d/stderr.txt" ]] && { echo "error: wsprd wrote to stderr in $d" >&2; exit 1; }
  return 0
}
rm -rf "$OUT"; mkdir -p "$OUT"
for c in "${CASES[@]}"; do
  IFS='|' read -r name opts seedcase <<< "$c"
  seed=""
  if [[ -n "${seedcase:-}" ]]; then seed="$OUT/$seedcase/hashtable.txt"; fi
  # An 11-character call in the bucket of K1ABC (6521): `<RYEEUQHAFKK>` does not fit unpk_'s
  # 13-byte callsign buffer, so wsprd does not print or subtract that decode (wsprd_utils.c:392).
  if [[ "$name" == t3_long ]]; then seed="$TMP/t3_long.hashtable"; printf ' 6521 RYEEUQHAFKK FN42\n' > "$seed"; fi
  # shellcheck disable=SC2086 # options are split on purpose
  run "$TMP/$name.a" "$seed" $opts; run "$TMP/$name.b" "$seed" $opts
  mkdir -p "$OUT/$name"; printf '%s\n' "$opts" > "$OUT/$name/args"
  if [[ -n "$seed" ]]; then cp "$seed" "$OUT/$name/hashtable_in.txt"; fi
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
